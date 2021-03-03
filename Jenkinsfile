#!groovy​

// only keep 100 builds to prevent disk usage from growing out of control
properties([
    buildDiscarder(logRotator(artifactDaysToKeepStr: '',
                              artifactNumToKeepStr: '',
                              daysToKeepStr: '',
                              numToKeepStr: '100'))])

//--------------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------------
void setBuildStatus(String message, String state) {
    // **NOTE** ManuallyEnteredCommitContextSource set to match the value used by bits of Jenkins outside pipeline control
    step([
        $class: "GitHubCommitStatusSetter",
        reposSource: [$class: "ManuallyEnteredRepositorySource", url: "https://github.com/BrainsOnBoard/bob_robotics"],
        contextSource: [$class: "ManuallyEnteredCommitContextSource", context: "continuous-integration/jenkins/branch"],
        errorHandlers: [[$class: "ChangingBuildStatusErrorHandler", result: "UNSTABLE"]],
        statusResultSource: [ $class: "ConditionalStatusResultSource", results: [[$class: "AnyBuildResult", message: message, state: state]] ]
    ]);
}

def runBuild(String name, String nodeLabel) {
    stage("Building " + name + " (" + env.NODE_NAME + ")") {
        dir(name) {
            // Delete CMake cache folder
            dir("build") {
                deleteDir();
            }

            // Generate unique name for message
            def uniqueMsg = "msg_build_" + name + "_" + env.NODE_NAME;

            setBuildStatus("Building " + name + " (" + env.NODE_NAME + ")", "PENDING");

            // Build tests and set build status based on return code
            def statusCode = sh script:"./build_all.sh -DGENN_PATH=\"" + WORKSPACE + "/genn\" 1> \"" + uniqueMsg + "\" 2> \"" + uniqueMsg + "\"", returnStatus:true
            if(statusCode != 0) {
                setBuildStatus("Building " + name + " (" + env.NODE_NAME + ")", "FAILURE");
            }

            // Archive output
            archive uniqueMsg;
        }
    }
}

//--------------------------------------------------------------------------
// Entry point
//--------------------------------------------------------------------------
// Build dictionary of available nodes and their labels
def availableNodes = [:]
for(node in jenkins.model.Jenkins.instance.nodes) {
    if(node.getComputer().isOnline() && node.getComputer().countIdle() > 0) {
        availableNodes[node.name] = node.getLabelString().split() as Set
    }
}

// Loop through all available nodes
def builderNodes = []
for(n in availableNodes) {
    // If this node supports opencv (variety is the spice of life w.r.t. testing bob_robotics)
    if("opencv" in n.value) {
        print "${n.key}";

        // Add node's name to list of builders and remove it from dictionary of available nodes
        // **YUCK** for some reason tuples aren't serializable so need to add an arraylist
        builderNodes.add([n.key, n.value])
    }
}

//--------------------------------------------------------------------------
// Parallel build step
//--------------------------------------------------------------------------
// **YUCK** need to do a C style loop here - probably due to JENKINS-27421
def builders = [:]
for(b = 0; b < builderNodes.size(); b++) {
    // **YUCK** meed to bind the label variable before the closure - can't do 'for (label in labels)'
    def nodeName = builderNodes.get(b).get(0)
    def nodeLabel = builderNodes.get(b).get(1).toString()

    // Create a map to pass in to the 'parallel' step so we can fire all the builds at once
    builders[nodeName] = {
        node(nodeName) {
            stage("Checking out project (" + env.NODE_NAME + ")") {
                checkout scm
            }

            stage("Downloading and building GeNN (" + env.NODE_NAME + ")") {
                sh 'bin/download_and_build_genn.sh'
            }

            runBuild("examples", nodeLabel);
            runBuild("projects", nodeLabel);
            runBuild("tools", nodeLabel);
            runBuild("tests", nodeLabel);

            // Parse test output for GCC warnings
            recordIssues enabledForFailure: true, tool: gcc(pattern: "**/msg_build_*_" + env.NODE_NAME, id: "gcc_" + env.NODE_NAME), qualityGates: [[threshold: 1, type: 'TOTAL', unstable: true]]

            stage("Running tests (" + env.NODE_NAME + ")") {
                setBuildStatus("Running tests (" + env.NODE_NAME + ")", "PENDING");
                dir("tests") {
                    // Generate unique name for message
                    def uniqueMsg = "msg_test_results_" + env.NODE_NAME;
                    def runTestsCommand = "./tests --gtest_output=xml:gtest_test_results.xml 1> \"" + uniqueMsg + "\" 2> \"" + uniqueMsg + "\"";
                    def runTestsStatus = sh script:runTestsCommand, returnStatus:true;

                    // Archive output
                    archive uniqueMsg;

                    // If tests failed, set failure status
                    if(runTestsStatus != 0) {
                        setBuildStatus("Running tests (" + env.NODE_NAME + ")", "FAILURE");
                    }
                }
            }

            stage("Gathering test results (" + env.NODE_NAME + ")") {
                junit '**/*_test_results.xml'
            }

            // Only run on nodes which actually have clang-tidy installed
            if(nodeLabel.contains("clang_tidy")) {
                stage("Running clang-tidy (" + env.NODE_NAME + ")") {
                    // Generate unique name for message
                    def uniqueMsg = "msg_clang_tidy_" + env.NODE_NAME;
                    def runClangTidyStatus = sh script:"./bin/run_clang_tidy_check 1> \"" + uniqueMsg + "\" 2> \"" + uniqueMsg + "\""

                    archive uniqueMsg;

                    recordIssues enabledForFailure: true, tool: clangTidy(pattern: "**/msg_clang_tidy_" + env.NODE_NAME, id: "clang_tidy_" + env.NODE_NAME), qualityGates: [[threshold: 1, type: 'TOTAL', unstable: true]]
                    if(runClangTidyStatus != 0) {
                        setBuildStatus("Running clang-tidy (" + env.NODE_NAME + ")", "FAILURE")
                    }
                }
            }
        }
    }
}

// Run builds in parallel
parallel builders
