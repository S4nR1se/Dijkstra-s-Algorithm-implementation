using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


public class CharacterController : MonoBehaviour
{
    const float ReachDistThreshold = 0.1f;
    const float CharacterMoveSpeed = 1.0f;
    protected Animator myAnimator;

    protected Grid.Tile myCurrentTile = null;
    protected bool myReachedTile = false;
    protected bool myReachedDestination = false;
    protected List<Grid.Tile> myWalkBuffer = new List<Grid.Tile>();

    Transform model = null;

    public virtual void UpdateCharacter()
    {
        Vector3 tilePosition = new Vector3();
        if (myCurrentTile != null) tilePosition = Grid.Instance.WorldPos(myCurrentTile);
        myReachedTile = Vector3.Distance(transform.position, tilePosition) < ReachDistThreshold;
        transform.position = Vector3.MoveTowards(transform.position, tilePosition, CharacterMoveSpeed * Time.deltaTime);

        myReachedDestination = myWalkBuffer.Count == 0;

        if (myWalkBuffer.Count > 0)
        {
            Grid.Tile t = myWalkBuffer.ElementAt(0);
            if (!t.occupied) MoveTile(myWalkBuffer.ElementAt(0));

            myAnimator.SetBool("Walk", true);
            SetForward((Grid.Instance.WorldPos(myWalkBuffer.ElementAt(0)) - myAnimator.transform.position).normalized);

            if (myReachedTile && myCurrentTile == t) myWalkBuffer.RemoveAt(0);
        }
        else if(myReachedTile)
        {
            myAnimator.SetBool("Walk", false);
        }
    }
    public virtual void StartCharacter() { }

    void Start()
    {
        myCurrentTile = Grid.Instance.GetClosest(transform.position);
        myAnimator = GetComponentInChildren<Animator>();

        model = transform.GetChild(0).transform;
    }
    void SetForward(Vector3 forward)
    {
        Vector3 newForward = forward;
        newForward.y = 0;


        model.forward = newForward.normalized;
    }
    public void MoveTile(Grid.Tile aTile)
    {
        if (myReachedTile && !aTile.occupied &&
            Grid.Instance.isReachable(myCurrentTile, aTile))
        {
            myCurrentTile = aTile;
        }
    }

    public void SetWalkBuffer(List<Grid.Tile> someTiles)
    {
        myWalkBuffer.Clear();
        myWalkBuffer.AddRange(someTiles);
    }
}
