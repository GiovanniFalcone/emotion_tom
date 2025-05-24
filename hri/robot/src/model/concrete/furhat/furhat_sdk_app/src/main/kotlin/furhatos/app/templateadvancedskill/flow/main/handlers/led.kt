package furhatos.app.templateadvancedskill.flow.main.handlers

import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpHandler
import furhatos.flow.kotlin.Flow
import furhatos.flow.kotlin.State
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.state
import org.json.JSONObject

/**
 * Class used to change color led.
 * It's asynchronous.
 */
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