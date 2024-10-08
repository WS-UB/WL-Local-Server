package com.example.wlmap
import LocationPermissionHelper
import android.R
import android.annotation.SuppressLint
import android.content.ContentValues
import android.content.Context
import android.content.Context.INPUT_METHOD_SERVICE
import android.content.Context.SENSOR_SERVICE
import android.content.res.Resources
import android.graphics.Color
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.util.Log
import android.view.Gravity
import android.view.LayoutInflater
import android.view.View
import android.view.MenuItem
import android.view.ViewGroup
import android.view.inputmethod.InputMethodManager
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.PopupWindow
import android.widget.RelativeLayout
import android.widget.SearchView
import android.widget.Spinner
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.ActionBarDrawerToggle
import androidx.appcompat.app.AppCompatActivity
import com.mapbox.common.location.AccuracyLevel
import com.mapbox.common.location.DeviceLocationProvider
import com.mapbox.common.location.IntervalSettings
import com.mapbox.common.location.Location
import com.mapbox.common.location.LocationObserver
import com.mapbox.common.location.LocationProviderRequest
import com.mapbox.common.location.LocationService
import com.mapbox.common.location.LocationServiceFactory
import com.mapbox.geojson.Point
import com.mapbox.geojson.Polygon
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.MapView
import com.mapbox.maps.RenderedQueryGeometry
import com.mapbox.maps.RenderedQueryOptions
import com.mapbox.maps.ScreenBox
import com.mapbox.maps.extension.style.expressions.generated.Expression
import com.mapbox.maps.extension.style.layers.generated.SymbolLayer
import com.mapbox.maps.extension.style.layers.generated.FillLayer
import com.mapbox.maps.extension.style.layers.getLayerAs
import com.mapbox.maps.extension.style.layers.properties.generated.LineJoin
import com.mapbox.maps.plugin.animation.flyTo
import com.mapbox.maps.plugin.annotation.AnnotationPlugin
import com.mapbox.maps.plugin.annotation.annotations
import com.mapbox.maps.plugin.annotation.generated.CircleAnnotation
import com.mapbox.maps.plugin.annotation.generated.CircleAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.CircleAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotation
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.createCircleAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.createPolylineAnnotationManager
import com.mapbox.maps.plugin.gestures.addOnMapClickListener
import com.mapbox.maps.plugin.gestures.gestures
import com.mapbox.maps.toCameraOptions
import org.eclipse.paho.client.mqttv3.MqttException
import java.lang.ref.WeakReference
import java.math.RoundingMode
import kotlin.math.round
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import androidx.appcompat.widget.Toolbar
import androidx.core.content.ContextCompat.getSystemService
import androidx.core.view.GravityCompat
import androidx.drawerlayout.widget.DrawerLayout
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentTransaction
import com.google.android.material.navigation.NavigationView
import okhttp3.Call
import okhttp3.Callback
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import java.io.IOException


class MapFragment : Fragment(),NavigationView.OnNavigationItemSelectedListener, SensorEventListener {
    private val serverUri = "tcp://128.205.218.189:1883" // Server address
    private val clientId = "001000"  // Client ID
    private val serverTopic = "receive-wl-map"  // ???
    private val STYLE_CUSTOM = "asset://style.json" // ???
    private val FLOOR1_LAYOUT = "davis01"
    private val FLOOR1_LABELS = "davis01labels"
    private val FLOOR1_DOORS = "davis01doors"
    private val FLOOR3_LABELS = "davis03labels"
    private val FLOOR3_LAYOUT = "davis03"
    private val FLOOR3_DOORS = "davis03doors"
    private val spinnerOptions = listOf("Select", "All", "Room", "Bathroom", "Staircase", "Elevator") // Drop down options
    private val LATITUDE = 43.0028 // Starting latitude
    private val LONGITUDE = -78.7873  // Starting longitude
    private val ZOOM = 17.9 // Starting zoom
    private val testUserLocation = Point.fromLngLat(-78.78755328875651, 43.002534795993796)


    private lateinit var mqttHandler: MqttHandler
    private lateinit var locationPermissionHelper: LocationPermissionHelper
    private lateinit var annotationAPI: AnnotationPlugin
    private lateinit var userAnnotationManager: CircleAnnotationManager
    private lateinit var doorAnnotationManager: CircleAnnotationManager
    private lateinit var pointAnnotationManager: CircleAnnotationManager
    private lateinit var polylineAnnotationManager: PolylineAnnotationManager
    private lateinit var mapView: MapView
    private lateinit var buttonF1: Button
    private lateinit var buttonF3: Button
    private lateinit var popupWindow: PopupWindow
    private lateinit var latAndlongWindow: PopupWindow

    private lateinit var sensorManager: SensorManager
    private lateinit var b :Button
    private lateinit var g: Button
    private lateinit var userLastLocation: Point

    //private var curRoute: List<Point> = null
    private var roomHighlighted = false
    private var doorSelected: Point? = null
    private var pointSelected: Point? = null
    private var circleAnnotationId: CircleAnnotation? = null
    private var prevRoute: PolylineAnnotation? = null
    private var routeDisplayed: Boolean = false //Determines if route is being displayed
    private var prevDoor: Boolean = false //Determines if prevDoor is being displayed
    private var prevPoint: Boolean = false
    private var lastLocation: Pair<Double, Double>? = null //Holds the longitude and latitude of the user's last location
    private var floorSelected: Int = 0 //Determines the floor selected (1-3)
    private lateinit var drawerLayout: DrawerLayout

    var runnable: Runnable = Runnable {
        initMQTTHandler()
    }

    // This function handles navigation item selections from a navigation drawer.
    // It overrides the 'onNavigationItemSelected' method of the NavigationView.OnNavigationItemSelectedListener interface.
    override fun onNavigationItemSelected(item: MenuItem): Boolean {
        return true
    }

    @SuppressLint("IncorrectNumberOfArgumentsInExpression")
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?): View? {

        super.onCreate(savedInstanceState)
        b = Button(requireContext())
        b.id = View.generateViewId() // Generate a unique id for the button
        g= Button(requireContext())
        g.id = View.generateViewId()
        //g.text="gyroscope"
        setUpSensor()

        // To start the MQTT Handler -- You must have:
        // 1. Server containers launched
        // 2. Connection to UB VPN or UB network
        val thread: Thread = Thread(runnable)
        thread.start()

        // Create a RelativeLayout to hold the MapView
        val container = RelativeLayout(requireContext())
        mapView = MapView(requireContext())

        // Start user LocationPuck plotting on and launching user NavigationRouting mapView
        userLocationPuck()

        // Initialize mapView to Davis Hall and set parameters
        initMapView()

        // Set ContentView to the RelativeLayout container
        container.addView(mapView)

        initManagers()

        // Initialize navigation directions popup
        initNavigationPopup()
        val readingbuttons = LinearLayout(requireContext())
        readingbuttons.id = View.generateViewId() // Generate a unique id for the LinearLayout
        val paramsButtons = RelativeLayout.LayoutParams(
            RelativeLayout.LayoutParams.WRAP_CONTENT,
            RelativeLayout.LayoutParams.WRAP_CONTENT
        )
        paramsButtons.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM)
        paramsButtons.addRule(RelativeLayout.ALIGN_PARENT_LEFT)
        paramsButtons.setMargins(16.dpToPx(), 16.dpToPx(), 16.dpToPx(), 80.dpToPx())
        readingbuttons.orientation = LinearLayout.VERTICAL
        readingbuttons.layoutParams = paramsButtons

        val buttonParams1 = LinearLayout.LayoutParams(
            300.dpToPx(),
            LinearLayout.LayoutParams.WRAP_CONTENT
        )
        buttonParams1.gravity = Gravity.END
        b.layoutParams = buttonParams1
        g.layoutParams
        readingbuttons.addView(b)
        readingbuttons.addView(g)
        container.addView(readingbuttons)

        // Initialize navigation directions popup
        initNavigationPopup()

        // Initializing floor selector and adding to ContentView container
        val floorLevelButtons = initFloorSelector()
        container.addView(floorLevelButtons)

        // Initializing drop down spinner and adding to ContentView container
        val spinner = initRoomSelector()
        container.addView(spinner)

        spinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onNothingSelected(parent: AdapterView<*>?) {
            }

            override fun onItemSelected(
                parent: AdapterView<*>?,
                view: View?,
                position: Int,
                id: Long
            ) {
                if (floorSelected == 0) {
                    return
                }
                if (spinnerOptions[position] == "All") {
                    if (floorSelected == 3) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf3 = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                            // Update layer properties
                            layerf3?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            symbolLayer?.filter(
                                Expression.neq(
                                    Expression.literal(""),
                                    Expression.literal("")
                                )
                            )
                            symbolLayer?.textField(
                                Expression.get("name"), // Existing text
                            )
                            layerf3?.fillColor("#7e7c77")
                            symbolLayer?.textColor(Color.parseColor("#000000"))
                        }
                    }else if (floorSelected == 1){
                        mapView.mapboxMap.getStyle { style ->
                            val layerf1 = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                            // Update layer properties
                            layerf1?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            symbolLayer?.filter(Expression.neq(Expression.literal(""), Expression.literal("")))
                            symbolLayer?.textField(
                                Expression.get("name"), // Existing text
                            )
                            layerf1?.fillColor("#7e7c77")
                            symbolLayer?.textColor(Color.parseColor("#000000"))
                        }
                    }
                } else if (spinnerOptions[position] == "Room") {
                    if (floorSelected == 3) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf3 = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                            // Update layer properties
                            layerf3?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            layerf3?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("room"), Expression.color(Color.parseColor("#A020F0")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.textAllowOverlap(true)
//                            symbolLayer?.textField(Expression.concat(
//                                Expression.get("name"), // Existing text
//                                Expression.literal(" room") // Additional string
//                            ))
                            symbolLayer?.filter(Expression.eq(Expression.literal("room"), Expression.get("type")))
                        }
                    }else if (floorSelected == 1){
                        mapView.mapboxMap.getStyle { style ->
                            val layerf1 = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                            // Update layer properties
                            layerf1?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf1?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("room"), Expression.color(Color.parseColor("#A020F0")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
//                            symbolLayer?.textField(Expression.concat(
//                                Expression.get("name"), // Existing text
//                                Expression.literal(" ROOM") // Additional string
//                            ))
                            symbolLayer?.filter(Expression.eq(Expression.literal("room"), Expression.get("type")))

                        }
                    }

                }else if(spinnerOptions[position] == "Bathroom"){
                    if (floorSelected == 3){
                        mapView.mapboxMap.getStyle { style ->
                            val layerf3 = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                            // Update layer properties
                            layerf3?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf3?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("bathroom"), Expression.color(Color.parseColor("#006400")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.filter(Expression.eq(Expression.literal("bathroom"), Expression.get("type")))

                        }
                    } else if (floorSelected == 1) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf1 = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                            // Update layer properties
                            layerf1?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf1?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("bathroom"), Expression.color(Color.parseColor("#006400")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.filter(Expression.eq(Expression.literal("bathroom"), Expression.get("type")))

                        }
                    }
                } else if (spinnerOptions[position] == "Staircase") {
                    if (floorSelected == 3) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf3 = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                            // Update layer properties
                            layerf3?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf3?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("stairwell"), Expression.color(Color.parseColor("#ADD8E6")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.filter(Expression.eq(Expression.literal("stairwell"), Expression.get("type")))
                        }
                    } else if (floorSelected == 1) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf1 = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                            // Update layer properties
                            layerf1?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf1?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("stairwell"), Expression.color(Color.parseColor("#ADD8E6")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.filter(Expression.eq(Expression.literal("stairwell"), Expression.get("type")))
                        }
                    }
                } else if (spinnerOptions[position] == "Elevator") {
                    if (floorSelected == 3) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf3 = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                            // Update layer properties
                            layerf3?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf3?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("elevator"), Expression.color(Color.parseColor("#C4A484")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.filter(Expression.eq(Expression.literal("elevator"), Expression.get("type")))
                        }
                    } else if (floorSelected == 1) {
                        mapView.mapboxMap.getStyle { style ->
                            val layerf1 = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                            // Update layer properties
                            layerf1?.fillOpacity(0.8)
                            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                            doorLayer?.iconOpacity(1.0)
                            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                            symbolLayer?.textOpacity(1.0)
                            symbolLayer?.textAllowOverlap(true)
                            layerf1?.fillColor(
                                Expression.match(
                                    Expression.get("type"), // Attribute to match
                                    Expression.literal("elevator"), Expression.color(Color.parseColor("#C4A484")), // Color for "room" polygons
                                    Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                                )
                            )
                            symbolLayer?.filter(Expression.eq(Expression.literal("elevator"), Expression.get("type")))
                        }
                    }
                }
            }

        }

        //searchbar

        val searchView = SearchView(requireContext())
        val layoutParams = RelativeLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            LinearLayout.LayoutParams.WRAP_CONTENT
        )
        layoutParams.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM)
        searchView.queryHint = "Search Room Name"
        searchView.isIconifiedByDefault = false
        searchView.setBackgroundColor(Color.DKGRAY)
        // change color
        // Set up 'X' button (close button) functionality
        val closeButtonId = searchView.context.resources.getIdentifier("android:id/search_close_btn", null, null)
        val closeButton = searchView.findViewById<ImageView>(closeButtonId)

        closeButton?.setOnClickListener {
            clearHighlightedRoom() // Clear both the room highlight and the navigation path
            searchView.setQuery("", false) // Clear the search bar text
            searchView.clearFocus() // Remove focus from the search bar
        }
        val id = searchView.context.resources
            .getIdentifier("android:id/search_src_text", null, null)
//        val closeButtonId = searchView.context.resources
//            .getIdentifier("android:id/search_close_btn", null, null)
        val magnifyId = searchView.context.resources.getIdentifier("android:id/search_mag_icon",null,null)
        val magnifyView = searchView.findViewById<View>(magnifyId) as ImageView
        magnifyView.setColorFilter(Color.WHITE)
        val buttonView = searchView.findViewById<View>(closeButtonId) as ImageView
        buttonView.setColorFilter(Color.WHITE)
        val textView = searchView.findViewById<View>(id) as TextView
        textView.setTextColor(Color.WHITE)
        textView.setHintTextColor(Color.WHITE)
        // Add the Searchbar to your layout
        container.addView(searchView, layoutParams)

        // Keep track of whether the search view is focused
        var isSearchViewFocused = false
        // Set an OnFocusChangeListener to the search view
        searchView.setOnQueryTextFocusChangeListener { _, hasFocus ->
            isSearchViewFocused = hasFocus
        }

        fun hideKeyboard(context: Context, view: View) {
            val inputMethodManager =
                context.getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager
            context.getSystemService(INPUT_METHOD_SERVICE) as InputMethodManager
            inputMethodManager.hideSoftInputFromWindow(view.windowToken, 0)
            searchView.clearFocus()
        }

        fun calculateCentroid(polygon: Polygon): Point {
            var lon = 0.0
            var lat = 0.0
            val len = polygon.coordinates()[0].size
            for (coordinate in polygon.coordinates()[0]){
                lon+=coordinate.longitude()
                lat+=coordinate.latitude()
            }
            return Point.fromLngLat(lon/len,lat/len)

        }

        searchView.setOnQueryTextFocusChangeListener { _, hasFocus ->
            if (hasFocus) {
                // The search bar has gained focus, recenter the map
                mapView.mapboxMap.flyTo(
                    CameraOptions.Builder()
                        .center(Point.fromLngLat(LONGITUDE, LATITUDE))
                        .pitch(0.0)
                        .zoom(ZOOM)
                        .bearing(0.0)
                        .build()
                )
            }
        }

        var userQuery = "" //Global variable that holds user search query (Room number)

        searchView.setOnQueryTextListener(object : SearchView.OnQueryTextListener {
            override fun onQueryTextSubmit(query: String): Boolean {
                clearHighlightedRoom()

                userQuery = query
                // This method will be called when the user submits the query (e.g., by pressing Enter)
                // You can perform your desired action here
                var sourceLayerId = ""
                var sourceLabelLayerId = ""
                var sourceLayerDoorId = ""
                if (floorSelected == 1) {
                    sourceLayerId = FLOOR1_LAYOUT
                    sourceLabelLayerId = FLOOR1_LABELS
                    sourceLayerDoorId = FLOOR1_DOORS
                } else if (floorSelected == 3) {
                    sourceLayerId = FLOOR3_LAYOUT
                    sourceLabelLayerId = FLOOR3_LABELS
                    sourceLayerDoorId = FLOOR3_DOORS
                }
                mapView.mapboxMap.getStyle { style ->
                    val layer = style.getLayerAs<FillLayer>(sourceLayerId)
                    // Update layer properties
                    layer?.fillOpacity(0.8)
                    val doorLayer = style.getLayerAs<SymbolLayer>(sourceLayerDoorId)
                    doorLayer?.iconOpacity(1.0)
                    val symbolLayer = style.getLayerAs<SymbolLayer>(sourceLabelLayerId)
                    symbolLayer?.textOpacity(1.0)
                    symbolLayer?.textAllowOverlap(true)
                    layer?.fillColor(
                        Expression.match(
                            Expression.get("name"), // Attribute to match
                            Expression.literal(query), Expression.color(Color.parseColor("#39ff14")), // Color for "room" polygons
                            Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                        )
                    )
                }

                mapView.mapboxMap.flyTo(
                    CameraOptions.Builder()
                        .center(Point.fromLngLat(LONGITUDE, LATITUDE))
                        .pitch(0.0)
                        .zoom(ZOOM)
                        .bearing(0.0)
                        .build()
                )
                val visibleBounds = mapView.mapboxMap.coordinateBoundsForCamera(CameraOptions.Builder()
                    .center(Point.fromLngLat(LONGITUDE, LATITUDE))
                    .pitch(0.0)
                    .zoom(ZOOM)
                    .bearing(0.0)
                    .build())

                val screenPoint1 = mapView.mapboxMap.pixelForCoordinate(visibleBounds.northwest())
                val screenPoint2 = mapView.mapboxMap.pixelForCoordinate(visibleBounds.southeast())
                val visibleAreaPolygon = ScreenBox(screenPoint1, screenPoint2)

                // Create a RenderedQueryGeometry from the visible area
                val renderedQueryGeometry = RenderedQueryGeometry(visibleAreaPolygon)
                val renderedQueryOptions = RenderedQueryOptions(
                    listOf(sourceLayerId),
                    Expression.eq(Expression.get("name"), Expression.literal(query))
                )

                // Query the room's feature and center on it
                mapView.mapboxMap.queryRenderedFeatures(
                    renderedQueryGeometry,
                    renderedQueryOptions
                ) { features ->
                    if (features.isValue) {
                        val f = features.value
                        if (f != null && f.size > 0) {
                            val room = f[0].queriedFeature.feature.geometry() as Polygon
                            val center = calculateCentroid(room) // Calculate room's center

                            // Set the pointSelected to the center of the room
                            pointSelected = center

                            // Only proceed if the pointSelected is valid
                            if (pointSelected != null) {
                                val screenPoint =
                                    mapView.mapboxMap.pixelForCoordinate(pointSelected!!)

                                // Make sure the screen point is valid and within bounds
                                if (screenPoint.x > 0 && screenPoint.y > 0) {
                                    // Show the popup at the selected room's location
                                    val x = screenPoint.x.toInt()
                                    val y = screenPoint.y.toInt()
                                    popupWindow.showAtLocation(searchView, Gravity.NO_GRAVITY, x, y)
                                } else {
                                    Log.e(
                                        "ERROR",
                                        "Invalid screen point: x=${screenPoint.x}, y=${screenPoint.y}"
                                    )
                                }
                            }
                        }
                    }
                }
                return true
            }

            override fun onQueryTextChange(newText: String): Boolean {
                // This method will be called when the text in the search view changes
                // You can implement any filtering logic here if needed
                return false
            }
        })



        mapView.mapboxMap.addOnMapClickListener { point ->

            var sourceLayerId = ""
            var sourceLabelLayerId = ""
            var sourceLayerDoorId = ""

            publishLocation(point)

            if (!isSearchViewFocused) {
                // If the search view is not focused, collapse it
                searchView.isIconified = true
            }

            // Hide the keyboard regardless of the focus state
            hideKeyboard(requireContext(), mapView)

            // Reset userQuery room color from search bar
            mapView.mapboxMap.getStyle { style ->
                if (floorSelected == 1) {
                    sourceLayerId = FLOOR1_LAYOUT
                    sourceLabelLayerId = FLOOR1_LABELS
                    sourceLayerDoorId = FLOOR1_DOORS
                } else if (floorSelected == 3) {
                    sourceLayerId = FLOOR3_LAYOUT
                    sourceLabelLayerId = FLOOR3_LABELS
                    sourceLayerDoorId = FLOOR3_DOORS
                }

                val layer = style.getLayerAs<FillLayer>(sourceLayerId)
                // Update layer properties
                layer?.fillOpacity(0.8)

                val doorLayer = style.getLayerAs<SymbolLayer>(sourceLayerDoorId)
                doorLayer?.iconOpacity(1.0)

                val symbolLayer = style.getLayerAs<SymbolLayer>(sourceLabelLayerId)
                symbolLayer?.textOpacity(1.0)

                symbolLayer?.textAllowOverlap(true)

                layer?.fillColor(
                    Expression.match(
                        Expression.get("name"), // Attribute to match
                        Expression.literal(userQuery), Expression.color(Color.parseColor("#7e7c77")), // Color for "room" polygons
                        Expression.color(Color.parseColor("#7e7c77")) // Default color for other polygons
                    )
                )
            }

            // Convert the geographic coordinates to screen coordinates
            val screenPoint = mapView.mapboxMap.pixelForCoordinate(point)
            val renderedQueryGeometry = RenderedQueryGeometry(screenPoint)
            val currentLayer = floorSelected
            if (currentLayer != 0) {
                if (currentLayer == 3) {
                    sourceLayerId = FLOOR3_LAYOUT
                } else if (currentLayer == 1) {
                    sourceLayerId = FLOOR1_LAYOUT
                }
                val renderedQueryOptions = RenderedQueryOptions(listOf(sourceLayerId), Expression.neq(Expression.literal(""), Expression.literal("")))
                mapView.mapboxMap.queryRenderedFeatures(renderedQueryGeometry, renderedQueryOptions) { features ->
                    if (features.isValue) {
                        val f = features.value
                        if (f != null && f.size > 0) {
                            val featureString = f[0].toString()
                            Log.d("DEBUG", featureString)
                            val propertiesIndex = featureString.indexOf("properties")
                            if (propertiesIndex != -1) {
                                var restOfTheString = featureString.substring(propertiesIndex+12)
                                val bracketIndex = restOfTheString.indexOf("}")
                                if (bracketIndex != -1) {
                                    restOfTheString = restOfTheString.substring(0, bracketIndex)
                                }
                                val finalString = restOfTheString.replace("\"", "").replace(",",", ").replace(":",": ")
                                Toast.makeText(requireContext(), finalString, Toast.LENGTH_SHORT ).show()

                                // Directions navigation popup on room click
                                val x = screenPoint.x.toInt()
                                val y = screenPoint.y.toInt()

                                //point the user selected
                                pointSelected = point

                                getClosestDoor(point,currentLayer) //Gets closest door and marks it with a circle

                                popupWindow.showAtLocation(searchView, Gravity.NO_GRAVITY, x, y)

                                var latitude = pointSelected!!.latitude().toBigDecimal().setScale(4, RoundingMode.UP).toString() //Convert latitude to a string rounded to the fourth decimal
                                var longitude = pointSelected!!.longitude().toBigDecimal().setScale(4, RoundingMode.UP).toString() //Convert longitude to a string rounded to the fourth decimal

                                val positionText = "(" + latitude + ", " + longitude + ")" //Set position text to the lat/long strings
                                initLatLongPopup(positionText) //Initialize the lat/long popup message with the positionText string
                                latAndlongWindow.showAtLocation(searchView, Gravity.NO_GRAVITY, x, y-100) //Show the lat/long popup message above the "Get Directions" popup


                                //Delete previously placed circles
                                pointAnnotationManager.deleteAll()

                                val circleMarkerOptions:CircleAnnotationOptions = CircleAnnotationOptions()
                                    .withPoint(pointSelected!!)
                                    .withCircleColor("#ffcf40") // Match the color with the polyline
                                    .withCircleRadius(7.0) // Set the radius of the circle
                                    .withCircleOpacity(1.0) // Set the opacity of the circle
                                    .withCircleSortKey(1.0) // Ensure the circle is drawn above the polyline

                                //create the circle on the user-selected point
                                pointAnnotationManager.create(circleMarkerOptions)
                                prevPoint = true
                            }
//                        val toast = Toast.makeText(this@MainActivity, print_m, Toast.LENGTH_LONG).show()
                        } else if (f == null || f.size == 0){

                            val x = screenPoint.x.toInt()
                            val y = screenPoint.y.toInt()

                            //point the user selected
                            pointSelected = point

                            popupWindow.showAtLocation(searchView, Gravity.NO_GRAVITY, x, y)

                            var latitude = pointSelected!!.latitude().toBigDecimal().setScale(4, RoundingMode.UP).toString() //Convert latitude to a string rounded to the fourth decimal
                            var longitude = pointSelected!!.longitude().toBigDecimal().setScale(4, RoundingMode.UP).toString() //Convert longitude to a string rounded to the fourth decimal

                            val positionText = "(" + latitude + ", " + longitude + ")" //Set position text to the lat/long strings
                            initLatLongPopup(positionText) //Initialize the lat/long popup message with the positionText string
                            latAndlongWindow.showAtLocation(searchView, Gravity.NO_GRAVITY, x, y-100) //Show the lat/long popup message above the "Get Directions" popup

                            //Delete previously placed circles
                            pointAnnotationManager.deleteAll()

                            val circleMarkerOptions:CircleAnnotationOptions = CircleAnnotationOptions()
                                .withPoint(pointSelected!!)
                                .withCircleColor("#ffcf40") // Match the color with the polyline
                                .withCircleRadius(7.0) // Set the radius of the circle
                                .withCircleOpacity(1.0) // Set the opacity of the circle
                                .withCircleSortKey(1.0) // Ensure the circle is drawn above the polyline

                            //create the circle on the user-selected point
                            pointAnnotationManager.create(circleMarkerOptions)
                            prevPoint = true
                            /*
                            mapView.mapboxMap.flyTo(
                                CameraOptions.Builder()
                                    .center(Point.fromLngLat(LONGITUDE, LATITUDE))
                                    .pitch(0.0)
                                    .zoom(ZOOM)
                                    .bearing(0.0)
                                    .build()
                            )
                             */

                        }
                    }
                }
            }
            true // Return true to consume the click event
        }

        buttonF1.setOnClickListener {
            floorSelected = 1

            if (routeDisplayed) { //If a route is already being displayed, delete the route and reset the routeDisplayed boolean
                polylineAnnotationManager.deleteAll()
                routeDisplayed = false
            }

            if (prevDoor) { //If a door is already being marked, delete the circle on the door and reset the prevDoor boolean
                doorAnnotationManager.deleteAll()
                prevDoor = false
            }

            if (prevPoint) { //If a point is already being marked, delete the circle on the point and reset the prevPoint boolean
                pointAnnotationManager.deleteAll()
                prevPoint = false
            }

            mapView.mapboxMap.getStyle { style ->
                // Get an existing layer by referencing its
                // unique layer ID (LAYER_ID)
                val layer = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                // Update layer properties
                layer?.fillOpacity(0.0)
                val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                doorLayer?.iconOpacity(0.0)
                // Add symbol layer for floor 3 labels
                val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                symbolLayer?.textOpacity(0.0)
            }
            mapView.mapboxMap.getStyle { style ->
                // Get an existing layer by referencing its
                // unique layer ID (LAYER_ID)
                val layer = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                // Update layer properties
                layer?.fillOpacity(0.8)
                val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                doorLayer?.iconOpacity(0.8)
                val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                symbolLayer?.textOpacity(1.0)
                symbolLayer?.textAllowOverlap(true)
                symbolLayer?.textFont(
                    listOf("DIN Offc Pro Bold") // Specify the font family with bold weight
                )
                symbolLayer?.textSize(Expression.interpolate {
                    exponential {
                        literal(2)
                    }
                    zoom()
                    stop {
                        literal(14)
                        literal(1)
                    }
                    stop {
                        literal(16)
                        literal(5)
                    }
                    stop {
                        literal(18)
                        literal(7)
                    }
                    stop {
                        literal(20)
                        literal(20)
                    }
                    stop {
                        literal(22)
                        literal(30)
                    }
                })
            }
        }


        buttonF3.setOnClickListener {
            floorSelected = 3

            if (routeDisplayed) { //If a route is already being displayed, delete the route and reset the routeDisplayed boolean
                polylineAnnotationManager.deleteAll()
                routeDisplayed = false
            }

            if (prevDoor) { //If a door is already being marked, delete the circle on the door and reset the prevDoor boolean
                doorAnnotationManager.deleteAll()
                prevDoor = false
            }

            if (prevPoint) { //If a point is already being marked, delete the circle on the point and reset the prevPoint boolean
                pointAnnotationManager.deleteAll()
                prevPoint = false
            }

            mapView.mapboxMap.getStyle { style ->
                // Get an existing layer by referencing its
                // unique layer ID (LAYER_ID)
                val layer = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                // Update layer properties
                layer?.fillOpacity(0.0)
                val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                doorLayer?.iconOpacity(0.0)
                val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                symbolLayer?.textOpacity(0.0)
            }
            mapView.mapboxMap.getStyle { style ->
                // Get an existing layer by referencing its
                // unique layer ID (LAYER_ID)
                val layer = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
                // Update layer properties
                layer?.fillOpacity(0.8)
                val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
                doorLayer?.iconOpacity(1.0)
                val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
                symbolLayer?.textOpacity(1.0)
                symbolLayer?.textAllowOverlap(true)
                symbolLayer?.textFont(
                    listOf("DIN Offc Pro Bold") // Specify the font family with bold weight
                )
                symbolLayer?.textSize(Expression.interpolate {
                    exponential {
                        literal(2)
                    }
                    zoom()
                    stop {
                        literal(14)
                        literal(1)
                    }
                    stop {
                        literal(16)
                        literal(5)
                    }
                    stop {
                        literal(18)
                        literal(7)
                    }
                    stop {
                        literal(20)
                        literal(20)
                    }
                    stop {
                        literal(22)
                        literal(30)
                    }
                })
            }
        }

        return container
    }

    private fun initManagers() {
        annotationAPI = mapView.annotations
        polylineAnnotationManager = annotationAPI.createPolylineAnnotationManager()
        userAnnotationManager = annotationAPI.createCircleAnnotationManager()
        doorAnnotationManager = annotationAPI.createCircleAnnotationManager()
        pointAnnotationManager = annotationAPI.createCircleAnnotationManager()
    }

    private fun userLocationPuck() {

        val locationService : LocationService = LocationServiceFactory.getOrCreate()
        val locationProvider: DeviceLocationProvider?

        val request = LocationProviderRequest.Builder()
            .interval(IntervalSettings.Builder().interval(0L).minimumInterval(0L).maximumInterval(0L).build())
            .displacement(2F)
            .accuracy(AccuracyLevel.HIGHEST)
            .build();

        val result = locationService.getDeviceLocationProvider(request)
        locationProvider = result.value

        val locationObserver = LocationObserver { locations ->
            //Log.e(ContentValues.TAG, "Location update received: $locations.")
            // Assuming you want to plot the first location received
            val location = updateLocation(locations[0].latitude,locations[0].longitude)
            val point = Point.fromLngLat(location.second,location.first)
            //Log.e(ContentValues.TAG, "Location update received: $location")

            // Set options for the resulting circle layer.
            val circleAnnotationOptions: CircleAnnotationOptions = CircleAnnotationOptions()

                // Define a geographic coordinate.
                .withPoint(point)

                // Style the circle that will be added to the map.
                .withCircleRadius(8.0)
                .withCircleColor("#4a90e2")
                .withCircleStrokeWidth(3.5)
                .withCircleStrokeColor("#FAF9F6")
                .withCircleSortKey(1.0)

            if (userAnnotationManager.annotations.contains(circleAnnotationId)) {
                // Delete the previous LocationPuck annotation
                userAnnotationManager.delete(circleAnnotationId!!)
            }

            // Store last location for nav routing algorithm
            userLastLocation = point

            // Create and add the new circle annotation to the map
            circleAnnotationId = userAnnotationManager.create(circleAnnotationOptions)
        }

        locationProvider?.addLocationObserver(locationObserver)
    }

    private fun initNavigationPopup() {
        // Create the button programmatically with an icon next to the text
        val button = Button(requireContext()).apply {
            text = "Get Directions"
            // Set the icon to the left of the text
            setCompoundDrawablesWithIntrinsicBounds(0, 0, R.drawable.ic_menu_directions, 0)
            setCompoundDrawablePadding(10) // Sets the padding to 10 pixels
            setOnClickListener {
                userNavigationRouting()
                popupWindow.dismiss()
            }
        }

        // Initialize the PopupWindow (assuming you have a PopupWindow instance)
        popupWindow = PopupWindow(requireContext()).apply {
            width = LinearLayout.LayoutParams.WRAP_CONTENT
            height = LinearLayout.LayoutParams.WRAP_CONTENT
            isFocusable = true
            contentView = button
            setBackgroundDrawable(null)
        }
    }

    // Place it near other helper functions like `initNavigationPopup()` for organization
    // Place it near other helper functions like `initNavigationPopup()` for organization
    private fun clearHighlightedRoom() {
        if (!roomHighlighted) return  // Exit if no room is highlighted

        mapView.mapboxMap.getStyle { style ->
            var sourceLayerId = ""
            var sourceLabelLayerId = ""
            var sourceLayerDoorId = ""

            if (floorSelected == 1) {
                sourceLayerId = FLOOR1_LAYOUT
                sourceLabelLayerId = FLOOR1_LABELS
                sourceLayerDoorId = FLOOR1_DOORS
            } else if (floorSelected == 3) {
                sourceLayerId = FLOOR3_LAYOUT
                sourceLabelLayerId = FLOOR3_LABELS
                sourceLayerDoorId = FLOOR3_DOORS
            }

            val layer = style.getLayerAs<FillLayer>(sourceLayerId)
            val symbolLayer = style.getLayerAs<SymbolLayer>(sourceLabelLayerId)
            val doorLayer = style.getLayerAs<SymbolLayer>(sourceLayerDoorId)

            layer?.fillColor(Expression.color(Color.parseColor("#7e7c77"))) // Reset color
            symbolLayer?.textOpacity(1.0)
            symbolLayer?.textAllowOverlap(true)

            popupWindow.dismiss()
            roomHighlighted = false  // Reset flag
        }
    }



    private fun initLatLongPopup(positionText: String) {
        // Create the button programmatically with an icon next to the text
        if (pointSelected != null){
            val button = Button(requireContext()).apply {
                text = positionText
                // Set the icon to the left of the text
//                setCompoundDrawablesWithIntrinsicBounds(0, 0, R.drawable.ic_menu_directions, 0)
//                setCompoundDrawablePadding(10) // Sets the padding to 10 pixels
                setCompoundDrawablesWithIntrinsicBounds(0, 0, R.drawable.ic_menu_compass, 0)
                setCompoundDrawablePadding(10) // Sets the padding to 10 pixels
                setOnClickListener {
                    latAndlongWindow.dismiss()
                }
            }

            // Initialize the PopupWindow (assuming you have a PopupWindow instance)
            latAndlongWindow = PopupWindow(requireContext()).apply {
                width = LinearLayout.LayoutParams.WRAP_CONTENT
                height = LinearLayout.LayoutParams.WRAP_CONTENT
                isFocusable = true
                contentView = button
                setBackgroundDrawable(null)
            }

        }
    }


    private fun initRoomSelector(): Spinner {
        // Create a Spinner
        val spinner = Spinner(requireContext())

        // Set up the adapter for the Spinner
        val adapter = ArrayAdapter(requireContext(), R.layout.simple_spinner_item, spinnerOptions)
        adapter.setDropDownViewResource(R.layout.simple_spinner_dropdown_item)
        spinner.adapter = object :
            ArrayAdapter<String>(requireContext(), R.layout.simple_spinner_dropdown_item, spinnerOptions) {
            override fun getDropDownView(
                position: Int,
                convertView: View?,
                parent: ViewGroup
            ): View {
                val view = super.getDropDownView(position, convertView, parent)
                val textView = view.findViewById<TextView>(R.id.text1)
                textView.setTextColor(Color.WHITE) // Set the desired text color here
                view.setBackgroundColor(Color.DKGRAY)
                return view
            }
            override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
                val view = super.getView(position, convertView, parent)
                val textView = view.findViewById<TextView>(R.id.text1)
                textView.setTextColor(Color.WHITE) // Set the desired text color here
                view.setBackgroundColor(Color.DKGRAY)
                return view
            }

        }

        // Set layout parameters for the Spinner
        val params = RelativeLayout.LayoutParams(
            RelativeLayout.LayoutParams.WRAP_CONTENT,
            RelativeLayout.LayoutParams.WRAP_CONTENT
        )

        params.addRule(RelativeLayout.ALIGN_PARENT_TOP) // Align to the top
        params.addRule(RelativeLayout.ALIGN_PARENT_END) // Align to the end (right)
        params.setMargins(16.dpToPx(), 16.dpToPx(), 60.dpToPx(), 16.dpToPx())

        spinner.layoutParams = params

        return spinner
    }

    private fun initFloorSelector(): LinearLayout {
        // Create a LinearLayout to hold the buttons
        val floorLevelButtons = LinearLayout(requireContext())
        floorLevelButtons.id = View.generateViewId() // Generate a unique id for the LinearLayout
        val paramsButtons = RelativeLayout.LayoutParams(
            RelativeLayout.LayoutParams.WRAP_CONTENT,
            RelativeLayout.LayoutParams.WRAP_CONTENT
        )
        paramsButtons.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM)
        paramsButtons.addRule(RelativeLayout.ALIGN_PARENT_END)
        paramsButtons.setMargins(16.dpToPx(), 16.dpToPx(), 16.dpToPx(), 80.dpToPx())
        floorLevelButtons.orientation = LinearLayout.VERTICAL
        floorLevelButtons.layoutParams = paramsButtons

        // Create and add buttons to the LinearLayout
        buttonF1 = Button(requireContext())
        buttonF1.id = View.generateViewId() // Generate a unique id for the button
        buttonF1.text = "1"
        buttonF1.setBackgroundColor(Color.DKGRAY)
        buttonF1.setTextColor(Color.WHITE)
        val buttonParams1 = LinearLayout.LayoutParams(
            50.dpToPx(),
            LinearLayout.LayoutParams.WRAP_CONTENT
        )
        buttonParams1.gravity = Gravity.END
        buttonF1.layoutParams = buttonParams1
        floorLevelButtons.addView(buttonF1)

        buttonF3 = Button(requireContext())
        buttonF3.id = View.generateViewId() // Generate a unique id for the button
        buttonF3.text = "3"
        buttonF3.setBackgroundColor(Color.DKGRAY)
        buttonF3.setTextColor(Color.WHITE)
        val buttonParams2 = LinearLayout.LayoutParams(
            50.dpToPx(),
            LinearLayout.LayoutParams.WRAP_CONTENT
        )
        buttonParams2.gravity = Gravity.END
        buttonF3.layoutParams = buttonParams2
        floorLevelButtons.addView(buttonF3)

        return floorLevelButtons
    }

    private fun initMapView() {
        // Enable gestures
        mapView.gestures.doubleTapToZoomInEnabled = true
        mapView.gestures.rotateEnabled = true
        mapView.gestures.pinchToZoomEnabled = true

        // Load custom on-device style
        mapView.mapboxMap.loadStyle(style = STYLE_CUSTOM)

        // Get and load the style for floor 1 of Davis Hall
        mapView.mapboxMap.getStyle { style ->
            val layer = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
            layer?.fillOpacity(0.0)

            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
            doorLayer?.iconOpacity(0.0)

            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
            symbolLayer?.textOpacity(0.0)
        }

        // Get and load styles for floor 3 of Davis Hall
        mapView.mapboxMap.getStyle { style ->
            val layer = style.getLayerAs<FillLayer>(FLOOR3_LAYOUT)
            layer?.fillOpacity(0.0)

            val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR3_DOORS)
            doorLayer?.iconOpacity(0.0)

            val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR3_LABELS)
            symbolLayer?.textOpacity(0.0)
        }

        // Set camera position to Davis Hall
        mapView.mapboxMap.setCamera(
            CameraOptions.Builder()
                .center(Point.fromLngLat(LONGITUDE, LATITUDE))
                .pitch(0.0)
                .zoom(ZOOM)
                .bearing(0.0)
                .build()
        )
    }

    private fun userNavigationRouting() {
        val navGraph = Graph()

        // HALL C117
        navGraph.addEdge(Point.fromLngLat(-78.7875593129828, 43.00265668269077),Point.fromLngLat(-78.78751469317484, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.00265668269077),Point.fromLngLat(-78.78742202301328, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78742202301328, 43.00265668269077),Point.fromLngLat(-78.78734166862753, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78734166862753, 43.00265668269077),Point.fromLngLat(-78.78724505207666, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78724505207666, 43.00265668269077),Point.fromLngLat(-78.78715369602253,43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78715369602253,43.00265668269077),Point.fromLngLat(-78.78705246522863, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78705246522863, 43.00265668269077),Point.fromLngLat(-78.78692752615112, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78692752615112, 43.00265668269077),Point.fromLngLat(-78.78689428375111, 43.00265668269077))

        // HALL C116
        navGraph.addEdge(Point.fromLngLat(-78.78689428375111, 43.00265668269077),Point.fromLngLat(-78.78689428375111, 43.00273843797882))
        navGraph.addEdge(Point.fromLngLat(-78.78689428375111, 43.00273843797882),Point.fromLngLat(-78.78689428375111, 43.002830102156025))
        navGraph.addEdge(Point.fromLngLat(-78.78689428375111, 43.002830102156025),Point.fromLngLat(-78.78689428375111, 43.00285059380832))

        // HALL C115 RIGHT
        navGraph.addEdge(Point.fromLngLat(-78.78689428375111, 43.00285059380832),Point.fromLngLat(-78.78695554257372, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78695554257372, 43.00285059380832),Point.fromLngLat(-78.78704131765394, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78704131765394, 43.00285059380832),Point.fromLngLat(-78.78708934724371, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78708934724371, 43.00285059380832),Point.fromLngLat(-78.78719371883047, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78719371883047, 43.00285059380832),Point.fromLngLat(-78.78724589506906, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78724589506906, 43.00285059380832),Point.fromLngLat(-78.78734049888922, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78734049888922, 43.00285059380832),Point.fromLngLat(-78.78742256376356, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78742256376356, 43.00285059380832),Point.fromLngLat(-78.78746825354492, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78746825354492, 43.00285059380832),Point.fromLngLat(-78.78751469317484, 43.00285059380832))

        // HALL C115 LEFT
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.00285059380832), Point.fromLngLat(-78.78765887996539, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78765887996539, 43.00285059380832),Point.fromLngLat(-78.78765887996539, 43.00283062635404))
        navGraph.addEdge(Point.fromLngLat(-78.78765887996539, 43.00285059380832),Point.fromLngLat(-78.78774185818665, 43.00285059380832))
        navGraph.addEdge(Point.fromLngLat(-78.78774185818665, 43.00285059380832),Point.fromLngLat(-78.78774185818665, 43.0028261742014))
        navGraph.addEdge(Point.fromLngLat(-78.78774185818665, 43.0028261742014),Point.fromLngLat(-78.78774185818665, 43.002802139447624))
        navGraph.addEdge(Point.fromLngLat(-78.78774185818665, 43.002802139447624),Point.fromLngLat(-78.78774185818665, 43.00277723594624))
        navGraph.addEdge(Point.fromLngLat(-78.78774185818665, 43.00277723594624),Point.fromLngLat(-78.78774185818665, 43.0027514136836))
        navGraph.addEdge(Point.fromLngLat(-78.78774185818665, 43.0027514136836),Point.fromLngLat(-78.78774185818665, 43.00272948399882))

        //HALL C102 MAIN
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.00285059380832),Point.fromLngLat(-78.78751469317484, 43.002776797223675))
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.002776797223675),Point.fromLngLat(-78.78758096654084, 43.002776797223675))
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.002776797223675),Point.fromLngLat(-78.78751469317484, 43.00270086610047))
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.00270086610047),Point.fromLngLat(-78.78751469317484, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.00270086610047),Point.fromLngLat(-78.7875593129828, 43.00265668269077))

        // HALL S103
        navGraph.addEdge(Point.fromLngLat(-78.78751469317484, 43.00270086610047),Point.fromLngLat(-78.78755275476179, 43.00270086610047))
        navGraph.addEdge(Point.fromLngLat(-78.78755275476179, 43.00270086610047), Point.fromLngLat(-78.7875593129828, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78755275476179, 43.00253315749791),Point.fromLngLat(-78.7875593129828, 43.00265668269077))
        navGraph.addEdge(Point.fromLngLat(-78.78755275476179, 43.00270086610047),Point.fromLngLat(-78.78755275476179, 43.00253315749791))
        navGraph.addEdge(Point.fromLngLat(-78.78755275476179, 43.00253315749791),Point.fromLngLat(-78.78755275476179, 43.002417555891725))
        navGraph.addEdge(Point.fromLngLat(-78.78755275476179, 43.002417555891725),Point.fromLngLat(-78.78766632742568, 43.002417555891725))
        navGraph.addEdge(Point.fromLngLat(-78.78766632742568, 43.002417555891725),Point.fromLngLat(-78.78773787669357, 43.002417555891725))

        // PROF RESEARCH ROOMS
        navGraph.addEdge(Point.fromLngLat(-78.78734049888922, 43.00285059380832),Point.fromLngLat(-78.78734108487438, 43.002889430792976))
        navGraph.addEdge(Point.fromLngLat(-78.78734108487438, 43.002889430792976),Point.fromLngLat(-78.78734108487438, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78734108487438, 43.002914153255716),Point.fromLngLat(-78.78736088745488, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78736088745488, 43.002914153255716),Point.fromLngLat(-78.78737682871366, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78736088745488, 43.002914153255716),Point.fromLngLat(-78.78739305144838, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78734108487438, 43.002914153255716),Point.fromLngLat(-78.78729926002813, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78729926002813, 43.002914153255716),Point.fromLngLat(-78.78728315802225, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78728315802225, 43.002914153255716),Point.fromLngLat(-78.78722184057364, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78722184057364, 43.002914153255716),Point.fromLngLat(-78.78714338536244, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78714338536244, 43.002914153255716),Point.fromLngLat(-78.78712877832045, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78712877832045, 43.002914153255716),Point.fromLngLat(-78.78706705691697, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78706705691697, 43.002914153255716),Point.fromLngLat(-78.78704951220875, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78704951220875, 43.002914153255716),Point.fromLngLat(-78.78698929803357, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78698929803357, 43.002914153255716),Point.fromLngLat(-78.78697330315416, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78697330315416, 43.002914153255716),Point.fromLngLat(-78.78695331355547, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78695331355547, 43.002914153255716),Point.fromLngLat(-78.78691174213813, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78691174213813, 43.002914153255716),Point.fromLngLat(-78.78689547894501, 43.002914153255716))
        navGraph.addEdge(Point.fromLngLat(-78.78689547894501, 43.002914153255716),Point.fromLngLat(-78.78683371458419, 43.002914153255716))

//        val test = listOf(
//            Point.fromLngLat(-78.78734108487438, 43.002914153255716)
//        )
//
//
//        circleAnnotationManager = annotationAPI.createCircleAnnotationManager()
//        for (point in test) {
//            // Create a circle marker for each point
//            val circleMarkerOptions: CircleAnnotationOptions = CircleAnnotationOptions()
//                .withPoint(point)
//                .withCircleColor("#000000") // Match the color with the polyline
//                .withCircleRadius(5.0) // Set the radius of the circle
//                .withCircleOpacity(1.0) // Set the opacity of the circle
//                .withCircleSortKey(1.0) // Ensure the circle is drawn above the polyline
//
//            // Add the circle marker to the map
//            circleAnnotationManager.create(circleMarkerOptions)
//        }

        if (doorSelected != null)
        {
            navGraph.walkPoints = grabWalk(navGraph)

            mapView.mapboxMap.getStyle { style ->
                // Get an existing layer by referencing its
                // unique layer ID (LAYER_ID)
                val layer = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                // Update layer properties
                layer?.fillOpacity(1.0)
                val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                doorLayer?.iconOpacity(0.0)
                // Add symbol layer for floor 3 labels
                val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                symbolLayer?.textOpacity(1.0)
            }

            userLastLocation = testUserLocation //Test for user location

            //Obtain the nearest point to the user and add the edge to navGraph
            val nearestUserPoint = navGraph.findClosestPoint(navGraph.walkPoints,userLastLocation)
            navGraph.addEdge(nearestUserPoint,userLastLocation)

            val nearestDoorPoint = navGraph.findClosestPoint(navGraph.walkPoints,doorSelected!!)
            navGraph.addEdge(nearestDoorPoint,doorSelected!!)

            val nearestPoint = navGraph.findClosestPoint(navGraph.walkPoints,pointSelected!!)
            navGraph.addEdge(nearestPoint,pointSelected!!)

            //A list that connects a path from the door point to the point selected within the room
            val door_to_roomPoint: List<Point> = listOf(doorSelected!!, pointSelected!!)


//            if (routeDisplayed) {
//                Log.e(ContentValues.TAG, "Deleting route annotations: ${polylineAnnotationManager.annotations}")
//                polylineAnnotationManager.deleteAll()
//
//            } else {
//                Log.e(ContentValues.TAG, "Route not displayed, not deleting annotations")
//            }

            if (routeDisplayed) {
                Log.e(ContentValues.TAG, "Deleting route annotations: ${polylineAnnotationManager.annotations}")
                polylineAnnotationManager.deleteAll()
                routeDisplayed = false
            }

            //Draws a path from the user location to the door
            val polylineAnnotationOptions_1: PolylineAnnotationOptions = PolylineAnnotationOptions()
                .withPoints(navGraph.calcRoute(userLastLocation, doorSelected!!))
                // Style the line that will be added to the map.
                .withLineColor("#0f53ff")
                .withLineWidth(6.3)
                .withLineJoin(LineJoin.ROUND)
                .withLineSortKey(0.0)

            prevRoute = polylineAnnotationManager.create(polylineAnnotationOptions_1)

            //Obtain the nearest point to the user-selected point and add the edge to navGraph

            //Draws a path from the door to the point selected by the user
            val polylineAnnotationOptions_2 = PolylineAnnotationOptions()
                .withPoints(door_to_roomPoint)
                // Style the line that will be added to the map.
                .withLineColor("#0f53ff")
                .withLineWidth(6.3)
                .withLineJoin(LineJoin.ROUND)
                .withLineSortKey(0.0)

            // Add the resulting line to the map.
            prevRoute = polylineAnnotationManager.create(polylineAnnotationOptions_2)
            routeDisplayed = true
            doorSelected = null
            return

        } else {
            navGraph.walkPoints = grabWalk(navGraph)

            mapView.mapboxMap.getStyle { style ->
                // Get an existing layer by referencing its
                // unique layer ID (LAYER_ID)
                val layer = style.getLayerAs<FillLayer>(FLOOR1_LAYOUT)
                // Update layer properties
                layer?.fillOpacity(1.0)
                val doorLayer = style.getLayerAs<SymbolLayer>(FLOOR1_DOORS)
                doorLayer?.iconOpacity(0.0)
                // Add symbol layer for floor 3 labels
                val symbolLayer = style.getLayerAs<SymbolLayer>(FLOOR1_LABELS)
                symbolLayer?.textOpacity(1.0)
            }

            userLastLocation = testUserLocation //Test for user location

            //Obtain the nearest point to the user and add the edge to navGraph
            val nearestUserPoint = navGraph.findClosestPoint(navGraph.walkPoints,userLastLocation)
            navGraph.addEdge(nearestUserPoint,userLastLocation)

            //Obtain the nearest point to the user-selected point and add the edge to navGraph
            val nearestPoint = navGraph.findClosestPoint(navGraph.walkPoints,pointSelected!!)
            navGraph.addEdge(nearestPoint,pointSelected!!)


//            if (routeDisplayed) {
//                Log.e(ContentValues.TAG, "Deleting route annotations: ${polylineAnnotationManager.annotations}")
//                polylineAnnotationManager.deleteAll()
//
//            } else {
//                Log.e(ContentValues.TAG, "Route not displayed, not deleting annotations")
//            }

            if (routeDisplayed) {
                Log.e(ContentValues.TAG, "Deleting route annotations: ${polylineAnnotationManager.annotations}")
                polylineAnnotationManager.deleteAll()
                routeDisplayed = false
            }



            val polylineAnnotationOptions: PolylineAnnotationOptions = PolylineAnnotationOptions()
                .withPoints(navGraph.calcRoute(userLastLocation, pointSelected!!))
                // Style the line that will be added to the map.
                .withLineColor("#0f53ff")
                .withLineWidth(6.3)
                .withLineJoin(LineJoin.ROUND)
                .withLineSortKey(0.0)

            // Add the resulting line to the map.


            prevRoute = polylineAnnotationManager.create(polylineAnnotationOptions)
            routeDisplayed = true
            pointSelected = null






        }



    }

    private fun grabWalk(graph: Graph): List<Point> {
        val walk = mutableListOf<Point>()

        for (node in graph.nodes) {
            walk.add(node.value.nodePoint)
        }

        return walk
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

    private fun findClosestPoint(points: List<Point>, userLocation: Point): Point {
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

    private fun getClosestDoor(point: Point, floor: Int) {
        val screenPoint = mapView.mapboxMap.pixelForCoordinate(point)
        val pointQueryGeometry = RenderedQueryGeometry(screenPoint)
        var doorLayerId = ""
        var layerId = ""
        if (floor != 0) {
            if (floor == 3) {
                doorLayerId = FLOOR3_DOORS
                layerId = FLOOR3_LAYOUT
            } else if (floor == 1) {
                doorLayerId = FLOOR1_DOORS
                layerId = FLOOR1_LAYOUT
            }
            val visibleBounds =
                mapView.mapboxMap.coordinateBoundsForCamera(mapView.mapboxMap.cameraState.toCameraOptions())

            val screenPoint1 = mapView.mapboxMap.pixelForCoordinate(visibleBounds.northwest())
            val screenPoint2 = mapView.mapboxMap.pixelForCoordinate(visibleBounds.southeast())
            val visibleAreaPolygon = ScreenBox(screenPoint1, screenPoint2)

            // Create a RenderedQueryGeometry from the visible area geometry
            val renderedQueryGeometry = RenderedQueryGeometry(visibleAreaPolygon)
            val doorQueryOptions = RenderedQueryOptions(
                listOf(doorLayerId),
                Expression.neq(Expression.literal(""), Expression.literal(""))
            )
            val layerQueryOptions = RenderedQueryOptions(
                listOf(layerId),
                Expression.neq(Expression.literal(""), Expression.literal(""))
            )
            var room = ""
            mapView.mapboxMap.queryRenderedFeatures(
                pointQueryGeometry,
                layerQueryOptions
            ) { features ->
                if (features.isValue) {
                    val f = features.value
                    if (f != null && f.size > 0) {
                        room = f[0].queriedFeature.feature.getProperty("name").toString()
                        room = room.substring(1, room.length - 1)
                        Log.e(ContentValues.TAG, "${room}")
                    }
                }
            }
            mapView.mapboxMap.queryRenderedFeatures(
                renderedQueryGeometry,
                doorQueryOptions
            ) { features ->
                if (features.isValue) {
                    val f = features.value
                    val l: MutableList<Point> = mutableListOf()
                    if (f != null && f.size > 0) {
                        //Log.e(ContentValues.TAG, "${f}")
                        var minDistance = Double.MAX_VALUE
                        for (feature in f) {
                            val door = feature.queriedFeature.feature.geometry() as Point
                            var roomsConnected =
                                feature.queriedFeature.feature.getProperty("room").toString()
                            roomsConnected = roomsConnected.substring(1, roomsConnected.length - 1)
                            val rooms = roomsConnected.split(",")
                            val startsWithC = rooms.any { room ->
                                room.trimStart().startsWith("C", ignoreCase = true)
                            }

                            if (rooms.contains(room) && startsWithC) {
                                Log.e(ContentValues.TAG, "DOOR FOUND")
                                val curDistance = haversine(door, point)
                                if (curDistance < minDistance) {
                                    //Log.e(ContentValues.TAG, "REASSIGNED AT: $curDistance")
                                    minDistance = curDistance
                                    doorSelected = door

                                    if (prevDoor) {
                                        doorAnnotationManager.deleteAll()
                                    }
                                    // Create a circle marker for each point
//                                    val circleMarkerOptions:CircleAnnotationOptions = CircleAnnotationOptions()
//                                        .withPoint(door)
//                                        .withCircleColor("#ffcf40") // Match the color with the polyline
//                                        .withCircleRadius(7.0) // Set the radius of the circle
//                                        .withCircleOpacity(1.0) // Set the opacity of the circle
//                                        .withCircleSortKey(1.0) // Ensure the circle is drawn above the polyline
//
//                                    // Add the circle marker to the map
//                                    doorAnnotationManager.create(circleMarkerOptions)
                                    prevDoor = true
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private fun updateLocation(newLatitude: Double, newLongitude: Double): Pair<Double, Double> {
        if (lastLocation == null) {
            lastLocation = Pair(newLatitude, newLongitude)
            return lastLocation!!
        }
        val alpha = 0.1 // Smoothing factor
        val latitude = lastLocation!!.first + alpha * (newLatitude - lastLocation!!.first)
        val longitude = lastLocation!!.second + alpha * (newLongitude - lastLocation!!.second)
        lastLocation = Pair(latitude, longitude)
        return lastLocation!!
    }

    private fun publishTestMessage(mqttHandler: MqttHandler) {
        val topic = "coordinates/topic" // Replace with your topic
        val testMessage = """{"latitude": 40.7128, "longitude": -74.0060}""" // Example JSON payload

        try {
            mqttHandler.publish(topic, testMessage)
            println("Test message published to $topic")
        } catch (e: MqttException) {
            println("Failed to publish test message: ${e.message}")
        }
    }

    class ElasticsearchHandler(private val esUrl: String) {
        private val okHttpClient = OkHttpClient()

        // Check document count
        fun checkDocumentCount(index: String) {
            val url = "$esUrl/$index/_count"
            val request = Request.Builder()
                .url(url)
                .build()

            okHttpClient.newCall(request).enqueue(object : Callback {
                override fun onFailure(call: Call, e: IOException) {
                    println("Error checking document count: ${e.message}")
                }

                override fun onResponse(call: Call, response: Response) {
                    response.use {
                        if (!response.isSuccessful) throw IOException("Unexpected code $response")

                        val responseBody = response.body?.string()
                        println("Document count in index '$index': $responseBody")
                    }
                }
            })
        }

        // Retrieve documents from Elasticsearch
        fun retrieveDocuments(index: String) {
            val url = "$esUrl/$index/_search?pretty"
            val request = Request.Builder()
                .url(url)
                .build()

            okHttpClient.newCall(request).enqueue(object : Callback {
                override fun onFailure(call: Call, e: IOException) {
                    println("Error retrieving documents: ${e.message}")
                }

                override fun onResponse(call: Call, response: Response) {
                    response.use {
                        if (!response.isSuccessful) throw IOException("Unexpected code $response")

                        val responseBody = response.body?.string()
                        println("Documents in index '$index': $responseBody")
                    }
                }
            })
        }
    }

    private fun initMQTTHandler() {
        val esUrl = "http://128.205.218.189:9200"  // Change this to 10.0.2.2 if using an emulator
        val esIndex = "mqtt-data"  // Index name in Elasticsearch
        mqttHandler = MqttHandler()
        mqttHandler.connect(serverUri, clientId, esUrl, esIndex)
        mqttHandler.subscribe("test/topic")
        mqttHandler.onMessageReceived = { message ->
            val server_runnable: Runnable = Runnable {
                Log.e("SERVER", message)
            }
            val thread: Thread = Thread(server_runnable)
            thread.start()
        }
        // Publish a test message
        publishTestMessage(mqttHandler)

        // Initialize ElasticsearchHandler
        val elasticsearchHandler = ElasticsearchHandler(esUrl)

        // Check the document count in Elasticsearch
        elasticsearchHandler.checkDocumentCount(esIndex)

        // Retrieve documents from Elasticsearch
        elasticsearchHandler.retrieveDocuments(esIndex)
    }

    private fun publishLocation(point: Point) {
        val lat = point.latitude()
        val long = point.longitude()
        val serverMessage = "ack,$long,$lat"
        mqttHandler.publish("test/topic",serverMessage)
    }
    override fun onDestroy() {
        sensorManager.unregisterListener(this)
        super.onDestroy()
        try {
            mqttHandler.disconnect()
        } catch (e: MqttException) {
            e.printStackTrace()
        }
    }

    private fun Int.dpToPx(): Int {
        val density = Resources.getSystem().displayMetrics.density
        return (this * density).toInt()
    }
    private fun setUpSensor() {
        // Use the correct context to get the system service
        sensorManager = requireActivity().getSystemService(Context.SENSOR_SERVICE) as SensorManager

        sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.also{
            sensorManager.registerListener(this,it,SensorManager.SENSOR_DELAY_UI, SensorManager.SENSOR_DELAY_NORMAL)
        }

        sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.also{
            sensorManager.registerListener(this,it,SensorManager.SENSOR_DELAY_UI, SensorManager.SENSOR_DELAY_NORMAL)
        }
    }


    override fun onSensorChanged(event: SensorEvent?) {
        if(event?.sensor?.type == Sensor.TYPE_ACCELEROMETER){
            val x=event.values[0]
            val y= event.values[1]
            val z= event.values[2]
            val t="accelerator: "
            val comma= ", "
            b.apply{
                text=t.plus(x).plus(comma).plus(y).plus(comma).plus(z)
            }
            mqttHandler.publish("test/topic", t.plus(x).plus(comma).plus(y).plus(comma).plus(z))
        }
        if(event?.sensor?.type == Sensor.TYPE_GYROSCOPE){
            val x=event.values[0]
            val y= event.values[1]
            val z= event.values[2]
            val t="gyroscope: "
            val comma= ", "
            g.apply{
                text=t.plus(x).plus(comma).plus(y).plus(comma).plus(z)
            }
            mqttHandler.publish("test/topic", t.plus(x).plus(comma).plus(y).plus(comma).plus(z))
        }
    }

    override fun onAccuracyChanged(p0: Sensor?, p1: Int) {
        return
    }

}