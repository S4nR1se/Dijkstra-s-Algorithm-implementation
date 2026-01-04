using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// _     __________    _    ______   __   ____ ___  ____  _____           
//| |   | ____/ ___|  / \  / ___\ \ / /  / ___/ _ \|  _ \| ____|          
//| |   |  _|| |  _  / _ \| |    \ V /  | |  | | | | | | |  _|            
//| |___| |__| |_| |/ ___ | |___  | |   | |__| |_| | |_| | |___           
//|_____|_____\____/_/   \_\____| |_|    \____\___/|____/|_____|          


// ____   ___    _   _  ___ _____    ____ _   _    _    _   _  ____ _____ 
//|  _ \ / _ \  | \ | |/ _ |_   _|  / ___| | | |  / \  | \ | |/ ___| ____|
//| | | | | | | |  \| | | | || |   | |   | |_| | / _ \ |  \| | |  _|  _|  
//| |_| | |_| | | |\  | |_| || |   | |___|  _  |/ ___ \| |\  | |_| | |___ 
//|____/ \___/  |_| \_|\___/ |_|    \____|_| |_/_/   \_|_| \_|\____|_____|

public class State : MonoBehaviour
{
    [HideInInspector] public StateMachine mySm = null;
    public virtual void EnterState() { }
    public virtual void UpdateState() { }
    public virtual void ExitState() { }
}

public class StateMachine : MonoBehaviour
{
    State myCurrentState;
    public List<State> myStates = new List<State>();

    public void InitializeStateMachine()
    {
        foreach(State s in myStates)
        {
            s.mySm = this;
        }
    }
    public void UpdateStateMachine()
    {
        myCurrentState?.UpdateState();
    }

    public void ChangeState<T>() where T : State
    {
        foreach (State s in myStates)
        {
            if (s is T)
            {
                myCurrentState?.ExitState();
                myCurrentState = s;
                myCurrentState.EnterState();
                return;
            }
        }
    }
}
