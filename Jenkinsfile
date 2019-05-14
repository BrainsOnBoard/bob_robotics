#!groovy​

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
    def nodeLabel = builderNodes.get(b).get(1)

    // Create a map to pass in to the 'parallel' step so we can fire all the builds at once
    builders[nodeName] = {
        node(nodeName) {
            stage("Checking out project (" + env.NODE_NAME + ")") {
                checkout scm
            }

            stage("Building examples (" + env.NODE_NAME + ")") {
                // Run automatic tests
                if (isUnix()) {
                    dir("examples") {
                        // Delete CMake cache folder
                        dir("build") {
                            deleteDir();
                        }

                        // Generate unique name for message
                        def uniqueMsg = "msg_" + env.NODE_NAME;

                        setBuildStatus("Building examples", "PENDING");

                        // Build tests and set build status based on return code
                        def statusCode = sh script:"./build_all.sh 1> \"" + uniqueMsg + "\" 2> \"" + uniqueMsg + "\"", returnStatus:true
                        if(statusCode != 0) {
                            setBuildStatus("Building examples", "FAILURE");
                        }

                        // Parse test output for GCC warnings
                        // **NOTE** driving WarningsPublisher from pipeline is entirely undocumented
                        // this is based mostly on examples here https://github.com/kitconcept/jenkins-pipeline-examples
                        // **YUCK** fatal errors aren't detected by the 'GNU Make + GNU C Compiler (gcc)' parser
                        // however JENKINS-18081 fixes this for
                        // the 'GNU compiler 4 (gcc)' parser at the expense of it not detecting make errors...
                        def parserName = ("mac" in nodeLabel) ? "Apple LLVM Compiler (Clang)" : "GNU compiler 4 (gcc)";
                        step([$class: "WarningsPublisher",
                            parserConfigurations: [[parserName: parserName, pattern: uniqueMsg]],
                            unstableTotalAll: '0', usePreviousBuildAsReference: true]);

                        // Archive output
                        archive uniqueMsg;
                    }
                }
            }
        }
    }
}

// Run builds in parallel
parallel builders
