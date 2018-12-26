#!/usr/bin/env python3
#coding: utf-8

import licant

licant.execute("../../cynematic.g.py")

licant.libs.include("linalg-v3")
licant.libs.include("nos")
licant.libs.include("gxx")

licant.cxx_application("target",
	sources=["main.cpp"],
	mdepends=[
		"cynematic",
		"linalg-v3", 
		"nos",
		"gxx"
	],	
)

licant.ex("target")