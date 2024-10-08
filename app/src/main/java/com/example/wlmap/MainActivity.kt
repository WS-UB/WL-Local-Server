package com.example.wlmap
import android.os.Bundle
import android.view.MenuItem
import android.widget.Toast
import androidx.appcompat.app.ActionBarDrawerToggle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.Toolbar
import androidx.core.view.GravityCompat
import androidx.drawerlayout.widget.DrawerLayout
import androidx.fragment.app.FragmentTransaction
import com.google.android.material.navigation.NavigationView


class MainActivity : AppCompatActivity(),NavigationView.OnNavigationItemSelectedListener {
    private lateinit var drawerLayout: DrawerLayout

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Set the layout for the activity. This loads the 'activity_main' layout resource.
        setContentView(com.example.wlmap.R.layout.activity_main)

        // Find the DrawerLayout by its ID and assign it to 'drawerLayout'.
        drawerLayout = findViewById(com.example.wlmap.R.id.drawer_layout)

        // Find the Toolbar by its ID and assign it to 'toolbar'.
        val toolbar = findViewById<Toolbar>(com.example.wlmap.R.id.toolbar)

        // Set the toolbar as the app's ActionBar, allowing it to function like a standard action bar.
        setSupportActionBar(toolbar)

        // Find the NavigationView (the menu within the drawer) by its ID and assign it to 'homeScreen'.
        // The 'setNavigationItemSelectedListener' sets the current activity as the listener for menu item selections.
        val homeScreen = findViewById<NavigationView>(com.example.wlmap.R.id.nav_view)
        homeScreen.setNavigationItemSelectedListener(this)

        // Create an ActionBarDrawerToggle to handle the opening and closing of the navigation drawer.
        // The toggle binds the drawer, the activity (this), and the toolbar, and sets labels for open and close states.
        val toggle = ActionBarDrawerToggle(this, drawerLayout, toolbar, com.example.wlmap.R.string.open_home, com.example.wlmap.R.string.close_home)
        drawerLayout.addDrawerListener(toggle)
        toggle.syncState()


        // Check if the activity is being recreated (such as after a screen rotation) by checking if 'savedInstanceState' is null.
        // If it's null, it means the activity is newly created, so we should initialize the home fragment and set the default menu item.
        if (savedInstanceState == null) {
            // Replace the current fragment with the HomeFragment.
            replaceFragment(HomeFragment())
            // Set the "Home" menu item as checked by default in the navigation drawer.
            homeScreen.setCheckedItem(com.example.wlmap.R.id.nav_home)
        }
    }

    // This function handles navigation item selections from a navigation drawer.
    // It overrides the 'onNavigationItemSelected' method of the NavigationView.OnNavigationItemSelectedListener interface.
    override fun onNavigationItemSelected(item: MenuItem): Boolean {
        when(item.itemId){
            // If the "Home" item is selected, replace the current fragment with 'HomeFragment'.
            com.example.wlmap.R.id.nav_home -> supportFragmentManager.beginTransaction().replace(com.example.wlmap.R.id.fragment_container, HomeFragment()).commit()

            // If the "WLMap" item is selected, we run the interactive map.
            com.example.wlmap.R.id.nav_wlmap -> supportFragmentManager.beginTransaction().replace(
                com.example.wlmap.R.id.fragment_container, MapFragment()).commit()

            // If the "Data" item is selected, replace the current fragment with 'SendDataFragment'.
            com.example.wlmap.R.id.nav_data -> supportFragmentManager.beginTransaction().replace(com.example.wlmap.R.id.fragment_container, SendDataFragment()).commit()

            // If the "Settings" item is selected, replace the current fragment with 'SettingsFragment'.
            com.example.wlmap.R.id.nav_setting -> supportFragmentManager.beginTransaction().replace(
                com.example.wlmap.R.id.fragment_container, SettingsFragment()).commit()

            // If the "Logout" item is selected, show a Toast message saying "Logout!".
            com.example.wlmap.R.id.nav_logout -> Toast.makeText(this, "Logout!", Toast.LENGTH_SHORT).show()
        }
        drawerLayout.closeDrawer(GravityCompat.START)
        return true
    }

    // A private method that replaces the current fragment with the provided HomeFragment.
    private fun replaceFragment(fragment: HomeFragment){
        // Begin a new fragment transaction.
        val transaction: FragmentTransaction = supportFragmentManager.beginTransaction()

        // Replace the fragment currently in the 'fragment_container' with the given 'fragment'.
        transaction.replace(com.example.wlmap.R.id.fragment_container, fragment)

        // Commit the transaction to apply the changes.
        transaction.commit()
    }

    // Override the 'onBackPressed' method to handle the back button behavior.
    override fun onBackPressed() {
        // Call the parent class's 'onBackPressed' method to perform any default back press behavior.
        super.onBackPressed()

        // Check if the navigation drawer is open on the start (left) side of the screen.
        // If so, we close it.
        if (drawerLayout.isDrawerOpen(GravityCompat.START)){
            drawerLayout.closeDrawer(GravityCompat.START)
        }
        // If not, handle the back press as normal using 'onBackPressedDispatcher'.
        else{
            onBackPressedDispatcher.onBackPressed()
        }
    }
}