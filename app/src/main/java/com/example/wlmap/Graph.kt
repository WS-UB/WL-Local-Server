package com.example.wlmap

import android.content.ContentValues
import android.util.Log
import com.mapbox.geojson.Point
import java.util.PriorityQueue
import kotlin.math.*


class Graph {
    lateinit var walkPoints : List<Point>
    val nodes = mutableMapOf<String, NavPoint>()
    private val edges = mutableMapOf<String, Double>()

    fun addEdge(point1: Point, point2: Point) {
        val P1Id = "${point1.longitude()},${point1.latitude()}"
        val P2Id = "${point2.longitude()},${point2.latitude()}"

        val P1Node = nodes.getOrPut(P1Id) { initNode(P1Id, point1) }
        val P2Node = nodes.getOrPut(P2Id) { initNode(P2Id, point2) }

        P1Node.neighbors.add(P2Node.id)
        P2Node.neighbors.add(P1Node.id)

        val edgeToId = "$P1Id,$P2Id"
        val edgeFromId = "$P2Id,$P1Id"
        val edgeDist = haversine(point1,point2)

        edges[edgeToId] = edgeDist
        edges[edgeFromId] = edgeDist

        /*
        Log.e(ContentValues.TAG, "EDGES: ${edges.size}")
        Log.e(ContentValues.TAG, "EDGES: ${P2Node.neighbors}")
         */
    }

    private fun initNode(pointID: String, point: Point): NavPoint {
        return NavPoint(id = pointID, nodePoint = point, neighbors = mutableSetOf())
    }

    private fun haversine(userLocation: Point, walkPoint: Point): Double {
        // Convert decimal degrees to radians
        val lon1Rad = Math.toRadians(userLocation.longitude())
        val lat1Rad = Math.toRadians(userLocation.latitude())
        val lon2Rad = Math.toRadians(walkPoint.longitude())
        val lat2Rad = Math.toRadians(walkPoint.latitude())

        // Haversine formula
        val dlon = lon2Rad - lon1Rad
        val dlat = lat2Rad - lat1Rad
        val a = sin(dlat / 2).pow(2.0) + cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2).pow(2.0)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return 6371 * c
    }

    fun findClosestPoint(points: List<Point>, userLocation: Point): Point {
        lateinit var closestPoint: Point
        var minDistance = Double.MAX_VALUE

        for (point in points) {
            val curDistance = haversine(userLocation,point)
            if (curDistance < minDistance) {
                minDistance = curDistance
                closestPoint = point
            }
        }

        return closestPoint
    }

    fun calcRoute(user: Point, door: Point): List<Point> {
        val pq = PriorityQueue<Pair<String, Double>>(compareBy { it.second })
        val distMap = mutableMapOf<String, Double>()
        val prevMap = mutableMapOf<String, String>()
        val visited = mutableSetOf<String>()
        val path = mutableListOf<Point>()

        // find node closest to the user
        val userId = "${user.longitude()},${user.latitude()}"
        val doorId = "${door.longitude()},${door.latitude()}"

        Log.e(ContentValues.TAG, "${nodes[doorId]?.neighbors}")


        // init nodes distMap and prevMap
        nodes.keys.forEach { node ->
            distMap[node] = if (node == userId) 0.0 else Double.MAX_VALUE
            pq.add(node to distMap[node]!!)
        }


        while(pq.isNotEmpty()) {
            val (currentId, currentDistance) = pq.poll()!!

            //Log.e(ContentValues.TAG, "${currentDistance}")

            if (currentId == doorId) {
                var currentNodeId = currentId
                while (currentNodeId != userId) {
                    currentNodeId = prevMap[currentNodeId]!!
                    path.add(nodes[currentNodeId]!!.nodePoint)
                }
                path.add(nodes[userId]!!.nodePoint)
                path.reverse()
                path.add(door)
                Log.e(ContentValues.TAG, "${path.contains(door)}")
                Log.e(ContentValues.TAG, "${nodes[doorId]?.neighbors}")
                return path
            }

            if (currentId !in visited) {
                visited.add(currentId)

                for (neighborId in nodes[currentId]!!.neighbors) {
                    val newDistance = currentDistance + edges["${currentId},${neighborId}"]!!
                    //Log.e(ContentValues.TAG, "${newDistance}")

                    if (newDistance < distMap.getOrDefault(neighborId, Double.MAX_VALUE)) {
                        distMap[neighborId] = newDistance
                        prevMap[neighborId] = currentId
                        pq.add(neighborId to newDistance)
                    }
                }
            }
        }

        return path
    }


}