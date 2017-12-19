FILE(REMOVE_RECURSE
  "CMakeFiles/clean_bindings"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clean_bindings.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
