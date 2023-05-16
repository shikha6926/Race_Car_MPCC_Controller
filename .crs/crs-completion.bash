_crs()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W "build clean test init generate -h --help" -- $cur) )
}
complete -F _crs crs-20
