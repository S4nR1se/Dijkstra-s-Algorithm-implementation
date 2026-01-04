using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//#     # #     #  #####  #     #    ######  ######     #    ### #     #  #####  
//#     # #     # #     # #     #    #     # #     #   # #    #  ##    # #     # 
//#     # #     # #       #     #    #     # #     #  #   #   #  # #   # #       
//#     # #     # #  #### #######    ######  ######  #     #  #  #  #  #  #####  
//#     # #     # #     # #     #    #     # #   #   #######  #  #   # #       # 
//#     # #     # #     # #     #    #     # #    #  #     #  #  #    ## #     # 
// #####   #####   #####  #     #    ######  #     # #     # ### #     #  #####  

public class Zombie : CharacterController
{
    float uuugh     =       0;
        Vector2 braaaaains = new Vector2(5, 8);
    Vector2Int          uhghbrains = new Vector2Int(3, 7);
    public override void StartCharacter()
    {
        myCurrentTile   =             Grid.Instance.GetClosest(transform.position);
        base.StartCharacter();
    }
    public override void UpdateCharacter()
    {
        base.UpdateCharacter();

        if (uuugh > 0)
        {
                         uuugh -= Time.deltaTime;
        }
                else
            {
                        ughBrainBrainugh();
        }
    }

    public void ughBrainBrainugh()
    {
        uuugh =  Random.Range(braaaaains.x,         braaaaains.y);
                        brainbrainzombiebrain();
    }

    public void brainbrainzombiebrain()
    {
            int         braaaaaains =           Random.Range(0, 4);
        int ughghhhhhh =    Random.Range(uhghbrains.x, uhghbrains.y);

        List<Grid.Tile>     brains =    new List<Grid.Tile>();

        Vector2Int bruhains =       Vector2Int.zero;

        switch (braaaaaains)
        {
                        case 0:
                bruhains =  new Vector2Int(1, 0 );
                break;
            case 1:
                bruhains = new          Vector2Int(-1, 0);
                break;
                            case 2:
                            bruhains = new Vector2Int(0, 1);
                break;
                case 3:
                bruhains = new Vector2Int(0, -1);
                    break;
        }

        for (int brainses = 1;              brainses < ughghhhhhh + 1; brainses++)
        {
                        Grid.Tile brainbrains = Grid.Instance.TryGetTile(new Vector2Int(myCurrentTile.x + (bruhains.x * brainses), myCurrentTile.y + (bruhains.y * brainses)));
            if (brainbrains != null)
            {
                    if (!brainbrains.occupied)
                {
                                        brains.Add(brainbrains);
                    }
            }
        }

            SetWalkBuffer(brains);
    }
}
