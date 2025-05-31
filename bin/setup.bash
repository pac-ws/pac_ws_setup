#!/usr/bin/env bash

_px4_completions()
{
    local cur prev opts long_opts excl_flags seen_flags \
          dir_opts name_opts world_opts sys_names

    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    opts="-d -n -c --gpu -b -x -s --headless -r --delete -h"
    long_opts="--directory --name --create --gpu --bash --xrce --sim \
--headless --world --robots --delete --help"

    dir_opts="-d --directory"
    name_opts="-n --name"
    world_opts="--world"

    # Exclusive flags
    excl_flags="-x --xrce -s --sim -r --robots -c --create -b --bash --delete"
    seen_flags=""
    for w in "${COMP_WORDS[@]}"; do
        for flag in ${excl_flags}; do
            if [[ "$w" == "$flag" ]]; then
                seen_flags="$seen_flags $flag"
            fi
        done
    done

    # Handle exlusive flags if used
    if [[ ${COMP_CWORD} -eq 1 ]]; then
        local available_opts="${long_opts} ${opts}"
        if [[ -n "${seen_flags}" ]]; then
            # Remove all other exclusive flags from suggestions
            for used in ${seen_flags}; do
                # strip exactly the used flag; (no-op if it's not in the list)
                available_opts="$(printf "%s\n" ${available_opts} | grep -vx "${used}")"
            done
            # Also remove any other exclusive flag that isn’t in seen_flags
            for ex in ${excl_flags}; do
                if ! grep -qx "${ex}" <<<"${seen_flags}"; then
                    available_opts="$(printf "%s\n" ${available_opts} | grep -vx "${ex}")"
                fi
            done
        fi
        COMPREPLY=( $(compgen -W "${available_opts}" -- "${cur}") )
        return 0
    fi

    # Directory completion
    if [[ " ${dir_opts} " == *" ${prev} "* ]]; then
        COMPREPLY=( $(compgen -d -- "${cur}") )
        return 0
    fi

    # Container name
    if [[ " ${name_opts} " == *" ${prev} "* ]]; then
        # Query Docker for running container names (one per line)
        sys_names=$(docker ps --format '{{.Names}}' 2>/dev/null)
        COMPREPLY=( $(compgen -W "${sys_names}" -- "${cur}") )
        return 0
    fi

    # world completion
    if [[ "${prev}" == "--world" ]]; then
        local worlds=( default baylands grid lawn )
        COMPREPLY=( $(compgen -W "${worlds[*]}" -- "${cur}") )
        return 0
    fi

    # dash completion
    if [[ "${cur}" == -* ]]; then
        local all_opts="${opts} ${long_opts}"
        local to_suggest="$all_opts"

        for w in "${COMP_WORDS[@]}"; do
            if [[ "${w}" == -* ]]; then
                to_suggest="$(printf "%s\n" ${to_suggest} | grep -vx "${w}")"
            fi
        done

        if [[ -n "${seen_flags}" ]]; then
            for ex in ${excl_flags}; do
                if ! grep -qx "${ex}" <<<"${seen_flags}"; then
                    to_suggest="$(printf "%s\n" ${to_suggest} | grep -vx "${ex}")"
                fi
            done
        fi

        COMPREPLY=( $(compgen -W "${to_suggest}" -- "${cur}") )
        return 0
    fi

    return 0
}

_pac_completions()
{
    local cur prev opts cmds sys_names
    COMPREPLY=()

    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    opts="-h -c -b"

    cmds="help create build mission rviz lpac bag list logs delete restart restart-px4 bash cmd \
gps batt journal update pose vel vlp vgps origin pac rqt"

    if [[ ${COMP_CWORD} -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "${opts} ${cmds}" -- "${cur}") )
        return 0
    fi

    case "${COMP_WORDS[1]}" in
        bag|list|logs|delete|restart|restart-px4|bash|gps|batt|journal|update|pose|vel|vlp|vgps)
            sys_names=$(docker ps --format '{{.Names}}' 2>/dev/null)
            COMPREPLY=( $(compgen -W "${sys_names}" -- "${cur}") )
            return 0
            ;;
        cmd)
            if [[ ${COMP_CWORD} -eq 2 ]]; then
                sys_names=$(docker ps --format '{{.Names}}' 2>/dev/null)
                COMPREPLY=( $(compgen -W "${sys_names}" -- "${cur}") )
                return 0
            else
                COMPREPLY=( $(compgen -c -- "${cur}") )
                return 0
            fi
            ;;
        *)
            ;;
    esac

    return 0
}

complete -F _pac_completions pac
complete -F _px4_completions px4

export_with_directory() {
  local var_name=$1
  local dir_name=$2

  if [ -n "${!var_name}" ]; then
    export "$var_name=$dir_name${!var_name:+:${!var_name}}"
  else
    export "$var_name=$dir_name"
  fi
}

export_with_directory PATH ${PAC_WS}/bin
