using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

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

public class Grid : MonoBehaviour
{
    #region Singleton
    public static Grid Instance;
    private void Awake()
    {
        if (Instance)
        {
            Debug.LogError("Grid already exists");
        }
        else
        {
            Instance = this;
        }
    }
    #endregion

    [SerializeField] int Width;
    [SerializeField] int Height;

    [Range(0.0f, 5.0f)]
    [SerializeField] float Spacing;

    [Range(0.0f, 1.0f)]
    [SerializeField] float VisualTileSize;

    [Range(0.0f, 1.0f)]
    [SerializeField] float AlphaTileSize;

    public bool GeneratedGrid = false;

    [SerializeField] Transform FinishTileTrans = null;

    [Serializable]
    public class Tile
    {
        public int x, y = 0;
        public bool occupied = false;
        public bool finishTile = false;
    }

    public List<Tile> tiles = new List<Tile>();

    [EditorCools.Button]
    void BakeGrid()
    {
        ClearGrid();

        for (int x = 0; x < Width; x++)
        {
            for (int y = 0; y < Height; y++)
            {

                Tile t = new Tile();
                t.x = x;
                t.y = y;
                t.occupied = false;

                Vector3 cubeSize = new Vector3(Spacing, 0.1f, Spacing);
                Vector3 cubePos = new Vector3();

                cubePos.x = (x * Spacing) + Spacing / 2.0f;
                cubePos.z = (y * Spacing) + Spacing / 2.0f;

                cubeSize /= 2;

                Collider[] cols = Physics.OverlapBox(cubePos + OffsetPos(), cubeSize);
                if (cols.Length > 0)
                {
                    foreach (Collider c in cols)
                    {
                        if (c.transform.CompareTag("Obstacle"))
                        {
                            t.occupied = true;
                            break;
                        }
                    }
                }

                tiles.Add(t);
            }
        }

        GetClosest(FinishTileTrans.position).finishTile = true;

        GeneratedGrid = true;
    }

    public Tile GetClosest(Tile aTile)
    {
        float dist = float.MaxValue;

        Vector3 aPosition = WorldPos(aTile);

        Tile returnTile = null;
        foreach (Tile t in tiles)
        {
            float curDist = Vector3.Distance(aPosition, WorldPos(t));
            if (curDist < dist && !t.occupied && !IsSameTile(t, aTile))
            {
                dist = curDist;
                returnTile = t;
            }
        }
        return returnTile;
    }

    public Tile TryGetTile(Vector2Int aPos)
    {
        Tile returnTile = null;

        foreach (Tile t in tiles)
        {
            if(t.x == aPos.x && t.y == aPos.y)
            {
                returnTile = t;
                break;
            }
        }

        return returnTile;
    }

    public Tile GetFinishTile()
    {
        foreach (Tile t in tiles)
        {
            if (t.finishTile) return t;
        }

        return null;
    }

    public List<Tile> GetTiles()
    {
        return tiles;
    }

    public Tile GetClosest(Vector3 aPosition)
    {
        float dist = float.MaxValue;
        Tile returnTile = null;
        foreach (Tile t in tiles)
        {
            float curDist = Vector3.Distance(aPosition, WorldPos(t));
            if (curDist < dist && !t.occupied)
            {
                dist = curDist;
                returnTile = t;
            }
        }
        return returnTile;
    }

    [EditorCools.Button]
    void ClearGrid()
    {
        GeneratedGrid = false;
        tiles.Clear();
    }

    private void OnDrawGizmos()
    {
        if (GeneratedGrid)
        {
            foreach (Tile t in tiles)
            {
                if (t.occupied) Gizmos.color = Color.red; else Gizmos.color = Color.green;
                if (t.finishTile) Gizmos.color = Color.blue;

                AlphaColor();
                Vector3 cubeSize = new Vector3(Spacing * VisualTileSize, 0.1f, Spacing * VisualTileSize);


                Vector3 cubePos = new Vector3();

                cubePos.x = (t.x * Spacing) + Spacing / 2.0f;
                cubePos.z = (t.y * Spacing) + Spacing / 2.0f;

                Gizmos.DrawWireCube(cubePos + OffsetPos(), cubeSize);
            }
        }
        else
        {
            for (int x = 0; x < Width; x++)
            {
                for (int y = 0; y < Height; y++)
                {
                    Gizmos.color = Color.yellow;
                    AlphaColor();

                    Vector3 cubeSize = new Vector3(Spacing * VisualTileSize, 0.1f, Spacing * VisualTileSize);
                    Vector3 cubePos = new Vector3();

                    cubePos.x = (x * Spacing) + Spacing / 2.0f;
                    cubePos.z = (y * Spacing) + Spacing / 2.0f;

                    Gizmos.DrawWireCube(cubePos + OffsetPos(), cubeSize);
                }
            }
        }
    }

    public bool isReachable(Tile from, Tile to)
    {
        Vector3 fromPos = WorldPos(from);
        Vector3 toPos = WorldPos(to);

        return Vector3.Distance(fromPos, toPos) < Spacing * MathF.Sqrt(2.1f);
    }

    public Vector3 WorldPos(Tile aTile)
    {
        Vector3 offset = new Vector3();

        offset.x = transform.position.x - (Spacing * Width) / 2;
        offset.z = transform.position.z - (Spacing * Height) / 2;
        offset.y = transform.position.y;

        Vector3 world = new Vector3();

        world.x = (aTile.x * Spacing) + Spacing / 2.0f;
        world.z = (aTile.y * Spacing) + Spacing / 2.0f;
        world.y = 0;

        return world + offset;
    }

    Vector3 OffsetPos()
    {
        Vector3 offset = new Vector3();

        offset.x = transform.position.x - (Spacing * Width) / 2;
        offset.z = transform.position.z - (Spacing * Height) / 2;
        offset.y = transform.position.y;

        return offset;
    }

    public bool IsSameTile(Tile aFirst, Tile aSecond)
    {
        if (aFirst.x == aSecond.x && aFirst.y == aSecond.y) return true; else return false;
    }

    void AlphaColor()
    {
        Color gcolor = Gizmos.color;
        gcolor.a = AlphaTileSize;
        Gizmos.color = gcolor;
    }

    public Vector3 GetWinPos()
    {
        return FinishTileTrans.position;
    }
}
