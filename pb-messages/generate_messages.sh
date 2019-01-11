#! /bin/bash

protoc --plugin=protoc-gen-nanopb=../nanopb/generator/protoc-gen-nanopb --nanopb_out=. *.proto
mv *.h ../proto_messages/
mv *.c ../proto_messages/
