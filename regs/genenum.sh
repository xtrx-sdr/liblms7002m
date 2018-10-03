#!/bin/bash
DIR="$( cd "$( dirname "$0" )" && pwd )"
set -x #echo on
python ${DIR}/enum_parser.py ${DIR}/*.json  > $1
