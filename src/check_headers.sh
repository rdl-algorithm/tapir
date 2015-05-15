files=$(find -name "*pp" -print)
need_doc=
for f in $files
do
    doc="$(grep -m 1 "^/\*\*" $f)"
    if [[ ${doc:0:10} != '/** @file ' ]]
    then
        need_doc+="$f "
    else
        name=${doc:10}
        if [[ $f == ${f%%$name} ]]
        then
            echo "WRONG NAME FOR $f: $name"
        fi
    fi
done
echo "MISSING DOC COMMENTS:"
for f in $need_doc
do
    echo $f
done
