using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

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

public class OutroState : State
{
    [SerializeField] GameObject WinUi;
    [SerializeField] GameObject LoseUi;

    [SerializeField] TextMeshProUGUI GameTimeText;
    [SerializeField] TextMeshProUGUI BurgerText;

    [SerializeField] Camera OutroCamera;

    [SerializeField] float Distance;
    [SerializeField] float Height;
    [SerializeField] float Speed;

    [SerializeField] string NextScene;

    private void Awake()
    {
        WinUi.SetActive(false);
        LoseUi.SetActive(false);    
    }
    public override void EnterState()
    {

        OutroCamera.gameObject.SetActive(true);
        if(Vector3.Distance(Grid.Instance.GetWinPos(), GamesManager.Instance.myKim.transform.position) < 1)
        {
            WinUi.SetActive(true);
            GameTimeText.text = "Total Time : " + GamesManager.Instance.GetTotlatGameTime.ToString("F3");

            BurgerText.text = GamesManager.Instance.GetCollectedBurgers.ToString() + "/" + GamesManager.Instance.GetBurgerCount.ToString() + "<sprite=0>";
        }
        else
        {
            LoseUi.SetActive(true);
        }
    }

    public override void ExitState()
    {

    }

    public override void UpdateState()
    {
        Vector3 Offset = new Vector3();
        Vector3 KimPos = GamesManager.Instance.myKim.transform.position;

        Offset.x = Mathf.Cos(Time.time * Speed) * Distance;
        Offset.z = Mathf.Sin(Time.time * Speed) * Distance;
        Offset.y = Height;

        Offset += KimPos;

        OutroCamera.transform.LookAt(KimPos);

        OutroCamera.transform.position = Offset;
    }

    public void NextLevel()
    {
        SceneManager.LoadScene(NextScene);
    }

    public void Restart()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }
}

