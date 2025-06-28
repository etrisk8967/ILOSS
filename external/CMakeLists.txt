# External dependencies CMakeLists.txt

# SQLite3 bundled version
if(NOT SQLite3_FOUND)
    set(SQLITE_VERSION "3440200")  # SQLite 3.44.2
    set(SQLITE_YEAR "2023")
    
    include(FetchContent)
    FetchContent_Declare(
        sqlite3
        URL https://www.sqlite.org/${SQLITE_YEAR}/sqlite-amalgamation-${SQLITE_VERSION}.zip
        URL_HASH SHA256=833be89b53b3be8b40a2e3d5fedb635080e3edb204957244f3d6987c2bb2345f
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    
    FetchContent_GetProperties(sqlite3)
    if(NOT sqlite3_POPULATED)
        FetchContent_Populate(sqlite3)
        
        # Create SQLite3 library
        add_library(sqlite3 STATIC
            ${sqlite3_SOURCE_DIR}/sqlite3.c
        )
        
        target_include_directories(sqlite3 PUBLIC ${sqlite3_SOURCE_DIR})
        
        # Set compile definitions
        target_compile_definitions(sqlite3 PRIVATE
            SQLITE_ENABLE_FTS5
            SQLITE_ENABLE_RTREE
            SQLITE_ENABLE_JSON1
            SQLITE_ENABLE_RBU
            SQLITE_ENABLE_STAT4
            SQLITE_ENABLE_COLUMN_METADATA
            SQLITE_OMIT_DEPRECATED
        )
        
        # Disable warnings for third-party code
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(sqlite3 PRIVATE -w)
        elseif(MSVC)
            target_compile_options(sqlite3 PRIVATE /w)
        endif()
        
        # Create an alias to match the system package
        add_library(SQLite::SQLite3 ALIAS sqlite3)
        set(SQLite3_FOUND TRUE CACHE BOOL "SQLite3 found" FORCE)
        set(SQLite3_VERSION "3.44.2" CACHE STRING "SQLite3 version" FORCE)
    endif()
endif()