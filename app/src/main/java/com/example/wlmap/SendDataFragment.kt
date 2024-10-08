package com.example.wlmap

import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText

class SendDataFragment : Fragment() {

    private lateinit var serverIdInput: EditText
    private lateinit var userIdInput: EditText
    private lateinit var passwordInput: EditText
    private lateinit var commentInput: EditText
    private lateinit var sendDataButton: Button

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_send_data, container, false)

        // Initialize views using the inflated layout
        serverIdInput = view.findViewById(R.id.fill_server_id)
        userIdInput = view.findViewById(R.id.fill_user_id)
        passwordInput = view.findViewById(R.id.fill_password)
        commentInput = view.findViewById(R.id.fill_comment)
        sendDataButton = view.findViewById(R.id.send_data_button)

        sendDataButton.setOnClickListener {
            val serverID = serverIdInput.text.toString()
            val userID = userIdInput.text.toString()
            val password = passwordInput.text.toString()
            val comment = commentInput.text.toString()

            // Print out the inputted information on the log to see if it's working or not.
            Log.i("Test Info", "Server ID: $serverID, User ID: $userID, Password: $password, Comment: $comment")
        }

        return view
    }
}