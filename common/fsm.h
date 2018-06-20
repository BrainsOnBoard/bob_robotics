#pragma once

// Standard C includes
#include <cassert>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// FSM
//----------------------------------------------------------------------------
// Finite-state machine class - makes logic of implementing robots etc much cleaner
template<typename S>
class FSM
{
public:
    //------------------------------------------------------------------------
    // StateHandler
    //------------------------------------------------------------------------
    class StateHandler
    {
    public:
        //--------------------------------------------------------------------
        // Enumerations
        //--------------------------------------------------------------------
        // Types of event that can occur within a state
        enum class Event
        {
            Enter,
            Exit,
            Update,
        };
        
        //--------------------------------------------------------------------
        // Declared virtuals
        //--------------------------------------------------------------------
        virtual bool handleEvent(S state, Event event) = 0;
    };
    
    FSM(StateHandler *stateHandler, S invalidState) 
        :   m_InvalidState(invalidState), m_StateHandler(stateHandler), m_CurrentState(invalidState),      
            m_NextState(invalidState), m_Updating(false)
    {
    }
    
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool update()
    {
        // Check a transition isn't already pending
        assert(m_NextState == m_InvalidState);
        
        // Set flag to show that current state is being update and update it
        m_Updating = true;
        const bool result = m_StateHandler->handleEvent(m_CurrentState, StateHandler::Event::Update);
        m_Updating = false;
        
        // If a transition was queued during update
        if(m_NextState != m_InvalidState) {
            // Make transition
            transition(m_NextState);
            
            // Invalidate next state
            m_NextState = m_InvalidState;
        }
        return result;
    }
    
    void transition(S state)
    {
        // If we're still updating current state
        if(m_Updating) {
            // After checking that there isn't already a transition queued, queue transition
            assert(m_NextState == m_InvalidState);
            m_NextState = state;
        }
        // Otherwise
        else {
            // If we are currently in a valid state, exit it
            if(m_CurrentState != m_InvalidState) {
                m_StateHandler->handleEvent(m_CurrentState, StateHandler::Event::Exit);
            }
            
            // Update current state and enter new state
            m_CurrentState = state;
            m_StateHandler->handleEvent(m_CurrentState, StateHandler::Event::Enter);
        }
    }
    
private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Which state enum value is used to signify 'invalid'
    const S m_InvalidState;
    
    // Class which handles state transitions
    StateHandler *m_StateHandler;
    
    // Current state
    S m_CurrentState;
    
    // State to transition to after update
    S m_NextState;
    
    // Are we mid way through a state update?
    bool m_Updating;
};
} // BoBRobotics
