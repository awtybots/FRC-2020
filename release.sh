#!/bin/sh

tag=$(git tag -l | head -n 1)
proj_name=$(ls ./build/libs/ | sed 's/\.jar//')
target_dir="./release-tmp/"

[ ! -d $target_dir/ ] &&
  mkdir $target_dir ||
  (echo "Please remove '$target_dir' and try again" &&
  rm -rf $target_dir && exit)

echo "Generating release for '$tag'"
git checkout $tag
./gradlew assemble

cp build/libs/* $target_dir/FRCUserProgram.jar

[ -d ./src/main/deploy/ ]  &&
  cp src/main/deploy/ -R $target_dir/

cd $target_dir
tar --gzip -cvf "../FRC-$proj_name-$tag.tgz" *
cd ..
rm -rf $target_dir
git checkout master
