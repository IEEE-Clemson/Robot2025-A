#!/bin/bash
source ./venv/bin/activate
nice -n -20 python -m tools.auto_demo || python -m tools.auto_demo
