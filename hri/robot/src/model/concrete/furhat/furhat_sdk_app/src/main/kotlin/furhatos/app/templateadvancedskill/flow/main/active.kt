package furhatos.app.templateadvancedskill.flow.main

import com.sun.net.httpserver.HttpServer
import furhatos.app.templateadvancedskill.flow.Parent
import furhatos.app.templateadvancedskill.flow.main.handlers.FeedbackHandler
import furhatos.app.templateadvancedskill.flow.main.handlers.GestureHandler
import furhatos.app.templateadvancedskill.flow.main.handlers.LedHandler
import furhatos.app.templateadvancedskill.flow.main.handlers.ListenHandler
import furhatos.app.templateadvancedskill.flow.main.handlers.SpeechHandler
import furhatos.app.templateadvancedskill.setting.AutoGlanceAway
import furhatos.app.templateadvancedskill.setting.beActive
import furhatos.flow.kotlin.*
import furhatos.flow.kotlin.voice.PollyVoice
import furhatos.util.Language
import java.net.InetSocketAddress

/**
 * State where Furhat engage actively with the user.
 * Start your interaction from here.
 */
val Active: State = state(Parent) {
    onEntry {
        furhat.voice = PollyVoice.Giorgio()
        furhat.setInputLanguage(
            Language.ITALIAN
        )

        furhat.beActive()

        val server = HttpServer.create(InetSocketAddress(9092), 0)
        server.createContext("/speech", SpeechHandler())
        server.createContext("/led", LedHandler())
        server.createContext("/feedback", FeedbackHandler())
        server.createContext("/gesture", GestureHandler())
        server.createContext("/listen", ListenHandler())
        server.executor = null
        server.start()

        println("Server started!")
    }

    include(AutoGlanceAway) // Glance away after some time of eye contact
    // include(AutoUserAttentionSwitching) // Switch user after a while
}