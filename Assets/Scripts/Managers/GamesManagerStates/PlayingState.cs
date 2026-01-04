using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
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

public class PlayingState : State
{
    List<CharacterController> controllerList;
    List<Burger> burgerList;
    CharacterController Kim;

    [SerializeField] Transform PlayingUI;

    [SerializeField] TextMeshProUGUI SpeedText;
    int Speed = 1;

    float GameTime = 0;

    private void Awake()
    {
        PlayingUI.gameObject.SetActive(false);
        controllerList = new List<CharacterController>();
        controllerList = FindObjectsOfType<CharacterController>(true).ToList();
        burgerList = FindObjectsOfType<Burger>(true).ToList();
    }
    public override void EnterState()
    {
        GameTime = 0;
        PlayingUI.gameObject.SetActive(true);
        Time.timeScale = 1;
        Speed = 1;
        SpeedText.text = ">";

        foreach (CharacterController c in controllerList)
        {
            c.StartCharacter();

            if (c is Kim)
            {
                Kim = c;
                break;
            }
        }
    }

    public override void ExitState()
    {
        Time.timeScale = 1;
        Speed = 1;
        SpeedText.text = ">";
        PlayingUI.gameObject.SetActive(false);
        GamesManager.Instance.SetTotalGameTime(GameTime);
        GameTime = 0;
    }

    public override void UpdateState()
    {
        GameTime += Time.deltaTime;
        if (!Kim) return;
        foreach (CharacterController c in controllerList)
        {
            c.UpdateCharacter();

            if (c != Kim)
            {
                float dist = Vector3.Distance(c.transform.position, Kim.transform.position);
                if (dist < 1)
                {
                    mySm.ChangeState<OutroState>();
                }
            }
        }

        foreach (Burger b in burgerList)
        {
            if (b.isActiveAndEnabled)
            {
                if (Vector3.Distance(b.transform.position, Kim.transform.position) < 1)
                {
                    GamesManager.Instance.CollectBurger();
                    b.gameObject.SetActive(false);
                }
            }
        }

        if (Vector3.Distance(GamesManager.Instance.myKim.transform.position, Grid.Instance.GetWinPos()) <= 1)
        {
            mySm.ChangeState<OutroState>();
        }
    }

    public void IncreaseSpeed()
    {
        Speed++;
        if (Speed > 3) Speed = 1;
        Time.timeScale = Speed;
        SpeedText.text = "";
        for (int i = 0; i < Speed; i++) SpeedText.text += ">";
    }
}
