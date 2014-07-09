#!/bin/bash
echo "Compile Boost 1.48?"
select yn in "Yes" "No"
do
    if [ "$yn" = "Yes" ]
    then
        cd tapir_boost_install/boost_1_48_0
        ./bootstrap.sh --prefix="$TAPIR_BOOST_148"
        echo "Building Boost."
        echo "This could take a long time..."
        sleep 1
        ./b2 install -j8
	cd ../..
	rm -rf tapir_boost_install
        exit 0
    else
        exit 1
    fi
done
