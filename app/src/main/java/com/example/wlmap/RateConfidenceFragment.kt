package com.example.wlmap

import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText
import android.widget.RatingBar
import androidx.fragment.app.DialogFragment


class RateConfidenceFragment : DialogFragment() {

    private lateinit var submitConfidenceRateButton: Button
    private lateinit var confidenceScore: RatingBar
    private lateinit var fillLocation: EditText

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_rate_confidence, container, false)

        submitConfidenceRateButton = view.findViewById(R.id.submit_rating_button)
        confidenceScore = view.findViewById(R.id.confidence_rating_bar)
        fillLocation = view.findViewById(R.id.fill_location)

        submitConfidenceRateButton.setOnClickListener{
            val locationText = fillLocation.text.toString()  // Get text from the EditText
            val ratingScore = confidenceScore.rating  // Get the rating from the RatingBar

            // Print out the rating and location (you can also use Log.i or Log.d)
            Log.i("Test Rating", "Location: $locationText, Rating Score: $ratingScore")
        }

        return view
    }

}