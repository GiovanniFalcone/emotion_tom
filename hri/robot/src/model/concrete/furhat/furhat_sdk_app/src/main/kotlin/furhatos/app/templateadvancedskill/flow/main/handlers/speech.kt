package furhatos.app.templateadvancedskill.flow.main.handlers

import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpHandler
import furhatos.flow.kotlin.Flow
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.state
import org.json.JSONObject

/**
 * Class used to utter anything (hints, greetings, ...)
 * It's blocking.
 */
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

/**
 * Class used in order to provide the feedback using prosody.
 */
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

                // sceglie il primo 'separatore' individuato
                val splitIndex = listOf(secondComma, firstDot, firstExcl)
                    .filter { it >= 0 }
                    .minOrNull() ?: -1

                val (firstPart, secondPart) = if (splitIndex != -1) {
                    val splitAt = splitIndex + 1
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
