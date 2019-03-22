#!/bin/sh
printf '$T'
printf "0: %.8x" $(( $1 & 0xffffffff)) | sed -E 's/0: (..)(..)(..)(..)/0: \4\3\2\1/' | xxd -r -g0
printf "0: %.4x" $(( $2 & 0xffff)) | sed -E 's/0: (..)(..)/0: \2\1/' | xxd -r -g0
printf '$T'
