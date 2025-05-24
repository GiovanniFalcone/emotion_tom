package furhatos.app.templateadvancedskill.flow.main.handlers

import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpHandler
import furhatos.app.templateadvancedskill.nlu.UserName
import furhatos.flow.kotlin.Flow
import furhatos.flow.kotlin.furhat
import furhatos.flow.kotlin.onNoResponse
import furhatos.flow.kotlin.onResponse
import furhatos.flow.kotlin.state
import furhatos.nlu.common.Yes

/**
 * Class used in order to listen anything.
 * It's blocking.
 */
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