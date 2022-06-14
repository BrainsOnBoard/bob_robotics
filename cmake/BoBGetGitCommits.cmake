macro(get_git_commit DIR VARNAME RV_VARNAME)
    find_package(Git)
    execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DIR}" rev-parse --short HEAD
                    RESULT_VARIABLE ${RV_VARNAME}
                    OUTPUT_VARIABLE ${VARNAME}
                    ERROR_VARIABLE unused
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(${RV_VARNAME} EQUAL 0)
        # Append -dirty if worktree has been modified
        execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DIR}" diff --no-ext-diff --quiet --exit-code
                        RESULT_VARIABLE git_dirty_rv)
        if(NOT ${git_dirty_rv} EQUAL 0)
            set(${VARNAME} ${${VARNAME}}-dirty)
        endif()
    else()
        # Git might fail (e.g. if we're not in a git repo), but let's carry on regardless
        set(${VARNAME} "(unknown)")
    endif()
endmacro()

macro(get_git_commits)
    # Pass the current git commits of project and BoB robotics as C macros
    get_git_commit("${CMAKE_SOURCE_DIR}" PROJECT_GIT_COMMIT rv)
    get_git_commit("${BOB_ROBOTICS_PATH}" BOB_ROBOTICS_GIT_COMMIT rv)

    if (${rv} EQUAL 0)
        # The git working tree is dirty
        if ("${BOB_ROBOTICS_GIT_COMMIT}" MATCHES dirty)
            set(GIT_TREE_DIRTY TRUE)
        endif()

        # Check whether the current commit is in master branch
        execute_process(COMMAND ${GIT_EXECUTABLE} -C "${DIR}" branch -a --contains ${BOB_ROBOTICS_GIT_COMMIT}
                        OUTPUT_VARIABLE ov
                        ERROR_VARIABLE ev)
        string(REGEX MATCH " (origin/)?master" match "${ov}")
        if("${match}" STREQUAL "")
            set(GIT_COMMIT_NOT_IN_MASTER TRUE)
        endif()
    endif()

    message(STATUS "BoB robotics git commit: ${BOB_ROBOTICS_GIT_COMMIT}")
    message(STATUS "Project git commit: ${PROJECT_GIT_COMMIT}")
endmacro(get_git_commits)
