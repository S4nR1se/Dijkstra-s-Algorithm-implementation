using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Kim : CharacterController
{
    [SerializeField] float ContextRadius;

    private List<Grid.Tile> _path;

    private Grid.Tile _currentTargetTile;
    private Grid.Tile _startTile;

    private const float AVOID_RADIUS = 2f;

    private const float FLEE_ENTER_RADIUS = 2.5f;
    private const float FLEE_EXIT_RADIUS = 5f;

    private float _nextRepathTime = 0f;
    private const float FLEE_REPATH_INTERVAL = 0.4f;

    private enum KimStates
    {
        Fleeing,
        Cruising
    }

    private KimStates _currentState = KimStates.Cruising;

    public override void StartCharacter()
    {
        _nextRepathTime = 0f;

        _currentState = KimStates.Cruising;

        if (myCurrentTile != null) return;
        myCurrentTile = Grid.Instance.GetClosest(transform.position);

        if (_startTile == null) _startTile = myCurrentTile;
    }

    public override void UpdateCharacter()
    {
        base.UpdateCharacter();

        Zombie closest = GetClosest(GetContextByTag("Zombie"))?.GetComponent<Zombie>();

        if (closest != null)
        {
            EvaluateBehavior(closest);
        }

        switch (_currentState)
        {
            case KimStates.Fleeing:
                Flee(closest);
                break;
            case KimStates.Cruising:
                Cruise();
                break;
        }

        if (_currentState == KimStates.Fleeing && _path != null && _path.Count > 0)
        {
            if (IsTileDangerous(_path[0]))
            {
                _path.Clear();
                myWalkBuffer.Clear();

                Flee(GetClosest(GetContextByTag("Zombie"))?.GetComponent<Zombie>());
            }
        }
    }

    private void EvaluateBehavior(Zombie closest)
    {
        if (closest == null)
        {
            _currentState = KimStates.Cruising;
            return;
        }

        float distance = Vector3.Distance(transform.position, closest.transform.position);

        switch (_currentState)
        {
            case KimStates.Cruising:
                if (distance < FLEE_ENTER_RADIUS)
                    _currentState = KimStates.Fleeing;
                break;

            case KimStates.Fleeing:
                if (distance > FLEE_EXIT_RADIUS)
                    _currentState = KimStates.Cruising;
                break;
        }
    }

    Vector3 GetEndPoint()
    {
        return Grid.Instance.WorldPos(Grid.Instance.GetFinishTile());
    }
    private bool IsTileDangerous(Grid.Tile tile)
    {
        Collider[] hits = Physics.OverlapSphere(
            Grid.Instance.WorldPos(tile),
            AVOID_RADIUS * 0.75f
        );

        foreach (Collider c in hits)
        {
            if (c.CompareTag("Zombie"))
                return true;
        }

        return false;
    }
    GameObject[] GetContextByTag(string aTag)
    {
        Collider[] context = Physics.OverlapSphere(transform.position, ContextRadius);
        List<GameObject> returnContext = new List<GameObject>();
        foreach (Collider c in context)
        {
            if (c.transform.CompareTag(aTag))
            {
                returnContext.Add(c.gameObject);
            }
        }
        return returnContext.ToArray();
    }

    GameObject GetClosest(GameObject[] aContext)
    {
        float dist = float.MaxValue;
        GameObject Closest = null;
        foreach (GameObject z in aContext)
        {
            float curDist = Vector3.Distance(transform.position, z.transform.position);
            if (curDist < dist)
            {
                dist = curDist;
                Closest = z;
            }
        }
        return Closest;
    }
    private void Cruise()
    {
        if (myReachedDestination || _path == null || _path.Count == 0)
        {
            Grid.Tile endTile = Grid.Instance.GetFinishTile();
            if (endTile != null && !Grid.Instance.IsSameTile(myCurrentTile, endTile))
            {
                EvaluatePath(endTile);
                if (_path != null && _path.Count > 0)
                {
                    SetWalkBuffer(_path);
                }
                else
                {
                    Grid.Tile fallback = Grid.Instance.GetClosest(transform.position);
                    EvaluatePath(fallback);
                }
            }
        }
    }
    private void Flee(Zombie closest)
    {
        if (closest == null) return;
        if (Time.time < _nextRepathTime) return;

        Grid.Tile safeTile = FindSafeTileTowardGoal(closest);
        if (safeTile == null) return;

        if (_currentTargetTile != safeTile || myWalkBuffer.Count == 0)
        {
            _currentTargetTile = safeTile;
            EvaluatePath(safeTile);

            if (_path != null && _path.Count > 0)
            {
                SetWalkBuffer(_path);
                _nextRepathTime = Time.time + FLEE_REPATH_INTERVAL;
            }
        }
    }

    private Grid.Tile FindSafeTileTowardGoal(Zombie zombie)
    {
        if (zombie == null) return null;

        Grid.Tile goalTile = Grid.Instance.GetFinishTile();
        if (goalTile == null) return null;

        List<Grid.Tile> tiles = Grid.Instance.GetTiles();
        if (tiles == null || tiles.Count == 0) return null;

        Grid.Tile bestTile = null;
        float bestScore = float.MinValue;

        Vector3 zombiePos = zombie.transform.position;
        Vector3 goalPos = Grid.Instance.WorldPos(goalTile);
        Vector3 currentPos = transform.position;

        foreach (Grid.Tile tile in tiles)
        {
            if (tile == null || tile.occupied) continue;
            if (tile == _startTile) continue;

            Vector3 tilePos = Grid.Instance.WorldPos(tile);
            float distFromZombie = Vector3.Distance(tilePos, zombiePos);

            if (distFromZombie < AVOID_RADIUS * 5f) continue;

            float distToGoal = Vector3.Distance(tilePos, goalPos);

            Vector3 fleeDir = (tilePos - zombiePos).normalized;
            Vector3 goalDir = (goalPos - currentPos).normalized;

            float alignment = Vector3.Dot(fleeDir, goalDir);

            float score = distFromZombie * 2f + alignment * 3f - distToGoal;

            if (score > bestScore)
            {
                bestScore = score;
                bestTile = tile;
            }
        }

        return bestTile ?? goalTile;
    }

    private void EvaluatePath(Grid.Tile endTile)
    {
        if (myCurrentTile == null || endTile == null) return;

        _path = Pathfinding(myCurrentTile, endTile);

        _path = Pathfinding(myCurrentTile, endTile);

        if (_path == null)
        {
            _path = new List<Grid.Tile>();
        }
    }
    private List<Grid.Tile> Pathfinding(Grid.Tile startTile, Grid.Tile endTile)
    {
        List<Grid.Tile> allTiles = Grid.Instance.GetTiles();
        Dictionary<Grid.Tile, float> distances = new Dictionary<Grid.Tile, float>();
        Dictionary<Grid.Tile, Grid.Tile> previous = new Dictionary<Grid.Tile, Grid.Tile>();
        List<Grid.Tile> unvisited = new List<Grid.Tile>();

        foreach (Grid.Tile tile in allTiles)
        {
            if (!tile.occupied)
            {
                distances[tile] = float.MaxValue;
                previous[tile] = null;
                unvisited.Add(tile);
            }
        }

        distances[startTile] = 0;

        while (unvisited.Count > 0)
        {
            Grid.Tile current = null;
            float minDist = float.MaxValue;

            foreach (Grid.Tile tile in unvisited)
            {
                if (distances[tile] < minDist)
                {
                    minDist = distances[tile];
                    current = tile;
                }
            }

            if (current == null || current == endTile) break;

            unvisited.Remove(current);

            List<Grid.Tile> neighbors = GetNeighbors(current);

            foreach (Grid.Tile neighbor in neighbors)
            {
                if (!unvisited.Contains(neighbor)) continue;
                if (IsTileDangerous(neighbor)) continue;
                if (neighbor == _startTile) continue;

                float altDistance = distances[current] + Vector3.Distance(
                    Grid.Instance.WorldPos(current),
                    Grid.Instance.WorldPos(neighbor));

                if (altDistance < distances[neighbor])
                {
                    distances[neighbor] = altDistance;
                    previous[neighbor] = current;
                }
            }
        }

        List<Grid.Tile> path = new List<Grid.Tile>();
        Grid.Tile step = endTile;

        if (previous[step] == null && !Grid.Instance.IsSameTile(step, startTile))
        {
            return null;
        }

        while (step != null && !Grid.Instance.IsSameTile(step, startTile))
        {
            path.Add(step);
            step = previous[step];
        }

        path.Reverse();
        return path;
    }
    private List<Grid.Tile> GetNeighbors(Grid.Tile tile)
    {
        List<Grid.Tile> neighbors = new List<Grid.Tile>();

        int[] dx = { -1, 0, 1, -1, 1, -1, 0, 1 };
        int[] dy = { -1, -1, -1, 0, 0, 1, 1, 1 };

        for (int i = 0; i < 8; i++)
        {
            Vector2Int neighborPos = new Vector2Int(tile.x + dx[i], tile.y + dy[i]);
            Grid.Tile neighbor = Grid.Instance.TryGetTile(neighborPos);

            if (neighbor != null && !neighbor.occupied && Grid.Instance.isReachable(tile, neighbor))
            {
                neighbors.Add(neighbor);
            }
        }

        return neighbors;
    }
    private void OnDrawGizmos()
    {
        if (_path == null) return;

        Gizmos.color = _currentState == KimStates.Cruising ? Color.green : Color.red;

        foreach (Grid.Tile tile in _path)
        {
            Gizmos.DrawSphere(Grid.Instance.WorldPos(tile), 0.1f);
        }
    }
}
