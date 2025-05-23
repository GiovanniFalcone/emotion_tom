package furhatos.app.templateadvancedskill.flow.main

import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpHandler
import furhatos.app.templateadvancedskill.nlu.UserName
import furhatos.flow.kotlin.Flow
import furhatos.flow.kotlin.State
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.onNoResponse
import furhatos.flow.kotlin.onResponse
import furhatos.flow.kotlin.state
import furhatos.gestures.Gestures
import furhatos.nlu.common.Yes
import org.json.JSONObject



class ListenHandler : HttpHandler {
    override fun handle(t: HttpExchange) {
        // get body
        val request = t.requestBody.bufferedReader().use { it.readText() }
        println(request)

        // response is sent here
        Flow().run(listenHandler(request, t))
    }
}

fun listenHandler(request: String, t: HttpExchange) = state {
    onEntry {
        println("Listen handler")
        furhat.listen(timeout = 5000)
    }

    // Personalizzato poichè per la classe in italiano dei nomi non c'è...
    // se ha domandato il nome e l'utente risponde con "il mio nome è @name"
    // invia il nome come risposta alla richiesta http
    onResponse<UserName> {
        val response = it.intent.name.toString()
        val os = t.responseBody
        t.sendResponseHeaders(200, response.length.toLong())
        os.write(response.toByteArray())
        println("Listen response sent: $response")
        os.close()

        terminate()
    }

    // qualsiasi sia la domanda, invia yes come risposta alla richiesta http
    onResponse<Yes>{
        // send yes
        val response = "yes"

        val os = t.responseBody
        t.sendResponseHeaders(200, response.length.toLong())
        os.write(response.toByteArray())
        println("Listen response sent: $response")
        os.close()

        terminate()
    }

    // tutto quello che ha ascoltato lo invia come risposta alla richiesta http
    onResponse {
        // get any answer
        val response = it.text.toString()

        val os = t.responseBody
        t.sendResponseHeaders(200, response.length.toLong())
        os.write(response.toByteArray())
        println("Listen response sent: $response")
        os.close()

        terminate()
    }

    onNoResponse {
        furhat.ask("Non ho capito, potresti ripetere?")
    }
}

class SpeechHandler : HttpHandler {
    override fun handle(t: HttpExchange) {
        val response = "This is the response: sentence uttered!"

        // get body
        val request = t.requestBody.bufferedReader().use { it.readText() }
        println(request)
        Flow().run(speechHandler(request))

        // response
        val os = t.responseBody
        t.sendResponseHeaders(200, response.length.toLong())
        os.write(response.toByteArray())
        println("Speech response sent")
        os.close()
    }
}
fun speechHandler(request: String) = state {
    onEntry {
        println("Speech handler")
        val hint = JSONObject(request).getString("sentence")
        furhat.say(hint)

        terminate()
    }
}

class FeedbackHandler : HttpHandler {
    override fun handle(t: HttpExchange) {
        val response = "This is the response"

        // get body
        val request = t.requestBody.bufferedReader().use { it.readText() }
        println(request)
        Flow().run(provideFeedback(request))

        // response
        val os = t.responseBody
        t.sendResponseHeaders(200, response.length.toLong())
        os.write(response.toByteArray())
        println("Feedback response sent")
        os.close()
    }
}

fun provideFeedback(request: String) = state {
    onEntry {
        /**
         * **********************************************************
         * Emotion      Pitch    Pitch_Range    Timing      Loudness
         * **********************************************************
         * Happiness    High       Large        Moderate    High
         * Surprise     High       Large        Slow        Moderate
         * Sadness      Low        Small        Slow        Low
         * Anger        High       Large        Fast        High
         * Disgust      Low        Small        Moderate    Low
         * Fear         High       Small        Fast        High
         * ************************************************************
         */

        println("ProvideFeedback")
        val message = JSONObject(request).getString("sentence")
        val emotion = JSONObject(request).getString("emotion")
        println(message)

        when (emotion) {
            "happy" -> furhat.say("<prosody pitch='high' rate='medium' volume='loud'>${message}</prosody>") // pitch='+30%'
            "neutral" -> furhat.say(message) // no prosody
            else -> {
                // val parts = message.split(",", limit = 2)
                val firstDot = message.indexOf('.')
                val firstExcl = message.indexOf('!')
                val commaIndices = message.indices.filter { message[it] == ',' }

                val secondComma = if (commaIndices.size >= 2) commaIndices[1] else -1

                // Scegli il primo indice valido (non -1) tra quelli trovati
                val splitIndex = listOf(secondComma, firstDot, firstExcl)
                    .filter { it >= 0 }
                    .minOrNull() ?: -1

                val (firstPart, secondPart) = if (splitIndex != -1) {
                    val splitAt = splitIndex + 1  // include il carattere di punteggiatura
                    val first = message.substring(0, splitAt).trim()
                    val second = message.substring(splitAt).trim()
                    first to second
                } else {
                    message to ""
                }

                println(firstPart)
                println(secondPart)

                furhat.say("""
                <prosody pitch='low' rate='slow' volume='soft'>${firstPart},</prosody>
                <prosody pitch='high' rate='fast' volume='medium'>${secondPart}</prosody> 
    
            """.trimIndent()) // pitch low rate low
            }
        }

        terminate()
    }
}

class LedHandler : HttpHandler {
    override fun handle(t: HttpExchange) {
        val response = "This is the response"
        t.sendResponseHeaders(200, response.length.toLong())
        // get body
        val request = t.requestBody.bufferedReader().use { it.readText() }
        Flow().runAsync(changeLed(request))
        // response
        val os = t.responseBody
        os.write(response.toByteArray())
        println("Led response sent")
        os.close()
    }
}

val changeLed: (String) -> State = { request ->
    state {
        onEntry {
            println(request)
            val r = JSONObject(request).getInt("r")
            val g = JSONObject(request).getInt("g")
            val b = JSONObject(request).getInt("b")

            furhat.ledStrip.solid(java.awt.Color(r, g, b))

            terminate()
        }
    }
}

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
