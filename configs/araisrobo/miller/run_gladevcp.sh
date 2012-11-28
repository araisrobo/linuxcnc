#!/bin/bash

nice gladevcp -c gladevcp \
	 -g 1024x1024+850+0 \
	 -u ./miller_vcp.py \
	 -H ./miller_vcp.hal \
	    ./miller_vcp.ui
