package com.example.wlmap

import com.mapbox.geojson.Point

data class NavPoint(val id: String, val nodePoint: Point, val neighbors: MutableSet<String> = mutableSetOf())
