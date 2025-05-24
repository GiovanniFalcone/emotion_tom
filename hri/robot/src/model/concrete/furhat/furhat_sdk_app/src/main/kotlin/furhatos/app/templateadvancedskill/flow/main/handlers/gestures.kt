package furhatos.app.templateadvancedskill.flow.main.handlers

import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpHandler
import furhatos.flow.kotlin.Flow
import furhatos.flow.kotlin.State
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.state
import furhatos.gestures.Gestures
import org.json.JSONObject

/**
 * Class used in order to do facial expressions.
 * It's asynchronous.
 */
class GestureHandler : HttpHandler {
    override fun handle(t: HttpExchange) {
        val response = "This is the response"
        t.sendResponseHeaders(200, response.length.toLong())
        // get body
        val request = t.requestBody.bufferedReader().use { it.readText() }
        Flow().runAsync(doGesture(request))
        // response
        val os = t.responseBody
        os.write(response.toByteArray())
        println("Gesture response sent")
        os.close()
    }
}

val doGesture: (String) -> State = { request ->
    state {
        onEntry {
            println(request)

            val gesture = JSONObject(request).getString("gesture")

            val gestures = Gestures.getGestureNames()
            if(gestures.contains(gesture)) {
                val index = gestures.indexOf(gesture);
                furhat.gesture(Gestures.getGestures()[index])
            } else {
                // custom gesture
                val filename = when {
                    gesture.equals("happy_1") -> "indefinitesmile_gesture"
                    gesture.equals("CustomSad") -> "sadtohappy_gesture"
                    else -> "surprise_gesture"
                }
                val gesture = Gestures.getResourceGesture("/${filename}.json")
                if (gesture != null) {
                    furhat.gesture(gesture = gesture)
                } else {
                    println("Gesture not found!")
                }
            }

            terminate()
        }
    }
}