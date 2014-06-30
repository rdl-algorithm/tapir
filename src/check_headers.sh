files=$(find -name "*pp" -print)
need_copyright=
need_doc=
for f in $files
do
    filename=$(basename $f)
    first_line="$(head -n 1 $f)"
    if [[ ${line:0:15} != "//    Copyright" ]]
    then
        need_copyright+="$f "
    fi

    doc="$(grep -m 1 "^/\*\*" $f)"
    if [[ ${doc:0:10} != '/** file: ' ]]
    then
        need_doc+="$f "
    else
        name=${doc:10}
        if [[ $name != $filename ]]
        then
            echo "WRONG NAME FOR $f: $name"
        fi
    fi
done
echo "MISSING COPYRIGHT NOTICES:"
for f in $need_copyright
do
    echo $f
done

echo
echo "MISSING DOC COMMENTS:"
for f in $need_doc
do
    echo $f
done
