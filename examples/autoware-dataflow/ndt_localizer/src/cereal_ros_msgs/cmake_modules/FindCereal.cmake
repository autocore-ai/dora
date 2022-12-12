# Try to find cereal's hpp files
find_path(CEREAL_DIRS 
    NAMES cereal 
    PATHS /usr/local/include /usr/include ../../
)

# find_library(CEREAL_LIBRARIES 
#     NAMES cereal 
#     PATHS /usr/local/include /usr/include ../../devel/lib/
# )