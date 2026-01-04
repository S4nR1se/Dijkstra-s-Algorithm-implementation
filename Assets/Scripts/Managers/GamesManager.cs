using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GamesManager : StateMachine
{
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

    #region Singleton
    public static GamesManager Instance;
    private void Awake()
    {
        Instance = this;
    }
    #endregion

    public GameObject myKim;
    float TotalGameTime;
    int BurgerCount;
    int CollectedBurgers;

    [SerializeField] bool SkipIntro = false;

    public Vector3 GetKimPos => myKim.transform.position;
    public float GetTotlatGameTime => TotalGameTime;
    public int GetBurgerCount => BurgerCount;
    public int GetCollectedBurgers => CollectedBurgers;

    private void Start()
    {
        InitializeStateMachine();
        BurgerCount = FindObjectsOfType<Burger>().Length;
        if (!SkipIntro)
        {
            ChangeState<IntroState>();
        }
        else
        {
            ChangeState<PlayingState>();
        }
    }

    private void Update()
    {
        UpdateStateMachine();
    }
    public void SetTotalGameTime(float aTime)
    {
        TotalGameTime = aTime;
    }
    public void CollectBurger()
    {
        CollectedBurgers++;
    }
}
