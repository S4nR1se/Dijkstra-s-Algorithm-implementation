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

public class IntroState : State
{
    [SerializeField] GameObject IntroSequence;
    [SerializeField] AnimationClip IntroClip;
    GameObject SpawnedIntro = null;
    private Animator myAnim;
    public override void EnterState()
    {
        GamesManager.Instance.myKim.SetActive(false);
        SpawnedIntro = Instantiate(IntroSequence, GamesManager.Instance.GetKimPos, Quaternion.identity);
        myAnim = SpawnedIntro.GetComponent<Animator>();
        Destroy(SpawnedIntro, IntroClip.length);
    }

    public override void ExitState()
    {

    }

    public override void UpdateState()
    {
        if (myAnim.GetCurrentAnimatorStateInfo(0).IsName("Finished"))
        {
            mySm.ChangeState<PlayingState>();
            GamesManager.Instance.myKim.SetActive(true);
        }
    }
}
