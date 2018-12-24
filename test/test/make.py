#!/usr/bin/env python3
#coding: utf-8

import licant

licant.execute("../../cynematic.g.py")

licant.libs.include("linalg-v3")
licant.libs.include("nos")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=[
		"cynematic",
		"linalg-v3", 
		"nos"
	],	
)

licant.ex("target")