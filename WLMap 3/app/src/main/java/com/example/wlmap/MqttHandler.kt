package com.example.wlmap

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken
import org.eclipse.paho.client.mqttv3.MqttCallback
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence

class MqttHandler {
    private var client: MqttClient? = null
    var onMessageReceived: ((String) -> Unit)? = null


    fun connect(brokerUrl: String?, clientId: String?) {
        if (brokerUrl.isNullOrEmpty() || clientId.isNullOrEmpty()) {
            println("Broker URL or Client ID cannot be null or empty")
            return
        }

        Thread {
            try {
                // Setup persistence
                val persistence = MemoryPersistence()

                // Initialize MQTT client
                client = MqttClient(brokerUrl, clientId, persistence)

                // Setup connection options
                val connectOptions = MqttConnectOptions()
                connectOptions.isCleanSession = true

                // Set callback
                client?.setCallback(object : MqttCallback {
                    override fun connectionLost(cause: Throwable?) {
                        // Handle connection loss
                    }

                    override fun messageArrived(topic: String?, message: MqttMessage?) {
                        message?.let {
                            val messageString = it.toString()
                            onMessageReceived?.invoke(messageString)
                        }
                    }

                    override fun deliveryComplete(token: IMqttDeliveryToken?) {
                        // Handle completed delivery
                    }
                })

                // Connect and subscribe
                client?.connect(connectOptions)
                client?.subscribe("coordinates/topic")

            } catch (e: MqttException) {
                e.printStackTrace()
            }
        }.start() // Run the connection in a background thread
    }

    fun disconnect() {
        try {
            client?.disconnect()
        } catch (e: MqttException) {
            e.printStackTrace()
        }
    }

    fun publish(topic: String?, message: String) {
        try {
            val mqttMessage = MqttMessage(message.toByteArray())
            client?.publish(topic, mqttMessage)
        } catch (e: MqttException) {
            e.printStackTrace()
        }
    }

    fun subscribe(topic: String?) {
        try {
            client!!.subscribe(topic)
        } catch (e: MqttException) {
            e.printStackTrace()
        }
    }

}