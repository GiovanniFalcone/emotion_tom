package furhatos.app.templateadvancedskill.flow

import furhat.libraries.standard.BehaviorLib.AutomaticMovements.randomHeadMovements
import furhatos.app.templateadvancedskill.flow.main.Idle
import furhatos.app.templateadvancedskill.setting.UniversalFallbackBehaviour
import furhatos.app.templateadvancedskill.setting.nextMostEngagedUser
import furhatos.flow.kotlin.*

/**
 * Parent state that house global definitions that should always be active in any state in the main flow
 */
val Global: State = state {
    include(UniversalFallbackBehaviour) //Set default fallback behaviour
}

/**
 * Additional more specific parent state.
 * Use for all states where the robot is being actively engaged in the interaction.
 */
val Parent: State = state(parent = Global) {
    // Random head movements can affect how gestures that include neck movements are carried out.
    // Either choose to not include random head movements at all
    // Or you can turn them on/off using enableVariableHeadMovements = false before performing the gesture
    include(randomHeadMovements())

    // Global users enter/leave behavior - override in the child state to set specific behaviour for the specific part of the interaction
    onUserEnter(instant = true) {
        when { // "it" is the user that entered
            furhat.isAttendingUser -> furhat.glance(it) // Glance at new users entering
            !furhat.isAttendingUser -> furhat.attend(it) // Attend user if not attending anyone
        }
        reentry()
    }

    onUserLeave(instant = true) {
        when { // "it" is the user that left
            !users.hasAny() -> goto(Idle) // no more users
            furhat.isAttending(it) -> furhat.attend(users.nextMostEngagedUser()) // current user left
            else -> furhat.glance(it.head.location) // other user left, just glance
        }
        reentry()
    }

    onExit(inherit = true) {
        // Reset local counters
        noMatch = 0
        noInput = 0
    }
}