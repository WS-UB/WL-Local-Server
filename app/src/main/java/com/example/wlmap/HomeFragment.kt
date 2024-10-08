package com.example.wlmap

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button

class HomeFragment : Fragment() {

    private lateinit var navigationButton: Button
    private lateinit var dataCollectionButton: Button

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_home, container, false)

        // Find buttons by their IDs
        navigationButton = view.findViewById(R.id.nav_button)
        dataCollectionButton = view.findViewById(R.id.data_collection_button)

        // Set onClickListener for navigationButton
        navigationButton.setOnClickListener {
            parentFragmentManager.beginTransaction()
                .replace(R.id.fragment_container, MapFragment())
                .commit()
        }

        // Set onClickListener for dataCollectionButton
        dataCollectionButton.setOnClickListener {
            parentFragmentManager.beginTransaction()
                .replace(R.id.fragment_container, DataCollectionFragment())
                .commit()
        }

        return view
    }
}