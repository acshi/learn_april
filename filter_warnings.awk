#!/usr/bin/awk -f
BEGIN {
    lines_to_skip = 0;
}

{
    if ($1 ~ /april2\/src/ && $2 ~ "warning:") {
        lines_to_skip = 2;
        next;
    }
    if (lines_to_skip > 0) {
        lines_to_skip--;
        next;
    }

    print;
}

END { }
