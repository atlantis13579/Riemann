#include <assert.h>
#include <memory.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <algorithm>
#include <limits>
#include <vector>
#include <map>

#include "GraphPartition.h"
#include "ConnectionGraph.h"

#if !defined(__GNUC__)
#pragma warning ( disable : 4018 )
#pragma warning ( disable : 5219 )
#endif

// METIS - Serial Graph Partitioning and Fill-reducing Matrix Ordering
// http://glaros.dtc.umn.edu/gkhome/metis/metis/overview
// https://github.com/KarypisLab/METIS
namespace metis
{
    typedef int idx_t;
    typedef float real_t;
    typedef void (*error_handler)(const char* msg);

#if !defined(__GNUC__)
    typedef int ssize_t;
#endif

    int METIS_PartGraphRecursive(idx_t* nvtxs, idx_t* ncon, idx_t* xadj,
        idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt,
        idx_t* nparts, real_t* tpwgts, real_t* ubvec, idx_t* options,
        idx_t* edgecut, idx_t* part, error_handler h);

    int METIS_PartGraphKway(idx_t* nvtxs, idx_t* ncon, idx_t* xadj,
        idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt,
        idx_t* nparts, real_t* tpwgts, real_t* ubvec, idx_t* options,
        idx_t* edgecut, idx_t* part, error_handler h);

    template<typename K, typename V>
    struct kv_t {
        K key;
        V val;
    };

    typedef kv_t<int, idx_t> ikv_t;
    typedef kv_t<float, idx_t> rkv_t;

    #define ASSERT(_exp)	assert(_exp);
    #define	iabs	abs
    #define	rabs	fabsf

    #define IDX_MAX   INT32_MAX
    #define IDX_MIN   INT32_MIN

    #define WCOREPUSH
    #define WCOREPOP

    #define LTERM                   (void **) 0
    #define HTLENGTH				((1<<13)-1)
    #define GK_MOPT_MARK            1
    #define GK_MOPT_CORE            2
    #define GK_MOPT_HEAP            3

    #define HTABLE_EMPTY            -1
    #define HTABLE_DELETED          -2
    #define HTABLE_FIRST             1
    #define HTABLE_NEXT              2

    #define UNMATCHED				-1
    #define LARGENIPARTS			7
    #define SMALLNIPARTS			5

    #define OMODE_REFINE            1
    #define OMODE_BALANCE           2

    #define VPQSTATUS_PRESENT      1
    #define VPQSTATUS_EXTRACTED    2
    #define VPQSTATUS_NOTPRESENT   3

    #define BNDTYPE_REFINE          1
    #define BNDTYPE_BALANCE         2

    #define INIT_MAXNAD             200

    #define AND(a, b) ((a) < 0 ? ((-(a))&(b)) : ((a)&(b)))
    #define OR(a, b) ((a) < 0 ? -((-(a))|(b)) : ((a)|(b)))
    #define XOR(a, b) ((a) < 0 ? -((-(a))^(b)) : ((a)^(b)))

    #define IFSET(a, flag, cmd) if ((a)&(flag)) (cmd);

    #define HASHFCT(key, size) ((key)%(size))
    #define SWAP gk_SWAP

    #define GETOPTION(options, idx, defval) ((options) == NULL || (options)[idx] == -1 ? (defval) : (options)[idx]) 
    #define I2RUBFACTOR(ufactor) (1.0f+0.001f*(ufactor))

    enum rstatus_et {
        METIS_OK = 1,
        METIS_ERROR_INPUT = -2,
        METIS_ERROR_MEMORY = -3,
        METIS_ERROR = -4
    };

    enum moptype_et {
        METIS_OP_PMETIS,
        METIS_OP_KMETIS,
        METIS_OP_OMETIS
    };

    enum moptions_et {
        METIS_OPTION_PTYPE,
        METIS_OPTION_OBJTYPE,
        METIS_OPTION_CTYPE,
        METIS_OPTION_IPTYPE,
        METIS_OPTION_RTYPE,
        METIS_OPTION_DBGLVL,
        METIS_OPTION_NIPARTS,
        METIS_OPTION_NITER,
        METIS_OPTION_NCUTS,
        METIS_OPTION_SEED,
        METIS_OPTION_ONDISK,
        METIS_OPTION_MINCONN,
        METIS_OPTION_CONTIG,
        METIS_OPTION_COMPRESS,
        METIS_OPTION_CCORDER,
        METIS_OPTION_PFACTOR,
        METIS_OPTION_NSEPS,
        METIS_OPTION_UFACTOR,
        METIS_OPTION_NUMBERING,
        METIS_OPTION_DROPEDGES,
        METIS_OPTION_NO2HOP,
        METIS_OPTION_TWOHOP,
        METIS_OPTION_FAST,

        /* Used for command-line parameter purposes */
        METIS_OPTION_HELP,
        METIS_OPTION_TPWGTS,
        METIS_OPTION_NCOMMON,
        METIS_OPTION_NOOUTPUT,
        METIS_OPTION_BALANCE,
        METIS_OPTION_GTYPE,
        METIS_OPTION_UBVEC
    };

    enum mptype_et {
        METIS_PTYPE_RB,
        METIS_PTYPE_KWAY
    };

    enum mctype_et {
        METIS_CTYPE_RM,
        METIS_CTYPE_SHEM
    };

    enum miptype_et {
        METIS_IPTYPE_GROW,
        METIS_IPTYPE_RANDOM,
        METIS_IPTYPE_EDGE,
        METIS_IPTYPE_NODE,
        METIS_IPTYPE_METISRB
    };

    enum mdbglvl_et {
        METIS_DBG_INFO = 1,       /*!< Shows various diagnostic messages */
        METIS_DBG_TIME = 2,       /*!< Perform timing analysis */
        METIS_DBG_COARSEN = 4,	  /*!< Show the coarsening progress */
        METIS_DBG_REFINE = 8,	  /*!< Show the refinement progress */
        METIS_DBG_IPART = 16, 	  /*!< Show info on initial partitioning */
        METIS_DBG_MOVEINFO = 32, 	  /*!< Show info on vertex moves during refinement */
        METIS_DBG_SEPINFO = 64, 	  /*!< Show info on vertex moves during sep refinement */
        METIS_DBG_CONNINFO = 128,     /*!< Show info on minimization of subdomain connectivity */
        METIS_DBG_CONTIGINFO = 256,     /*!< Show info on elimination of connected components */
        METIS_DBG_MEMORY = 2048     /*!< Show info related to wspace allocation */
    };


    /* Types of objectives */
    enum mobjtype_et {
        METIS_OBJTYPE_CUT,
        METIS_OBJTYPE_VOL,
        METIS_OBJTYPE_NODE
    };

    /* Default ufactors for the various operational modes */
    #define PMETIS_DEFAULT_UFACTOR          1
    #define MCPMETIS_DEFAULT_UFACTOR        10
    #define KMETIS_DEFAULT_UFACTOR          30
    #define OMETIS_DEFAULT_UFACTOR          200

    void ikvsorti(size_t n, ikv_t* base)
    {
        std::sort(base, base + 3, [](const ikv_t& a, const ikv_t& b) { return a.key < b.key; });
    }

    void ikvsortd(size_t n, ikv_t* base)
    {
        std::sort(base, base + 3, [](const ikv_t& a, const ikv_t& b) { return a.key > b.key; });
    }

    void isortd(size_t n, int* base)
    {
        std::sort(base, base + 3, [](int a, int b) { return a > b; });
    }

    void rkvsortd(size_t n, rkv_t* base)
    {
        std::sort(base, base + 3, [](const rkv_t& a, const rkv_t& b) { return a.key > b.key; });
    }

    #define ListInsert(n, lind, lptr, i) \
       do { \
         assert(lptr[i] == -1); \
         lind[n] = i; \
         lptr[i] = (n)++;\
       } while(0) 

    #define ListDelete(n, lind, lptr, i) \
       do { \
         assert(lptr[i] != -1); \
         lind[lptr[i]] = lind[--(n)]; \
         lptr[lind[n]] = lptr[i]; \
         lptr[i] = -1; \
       } while(0) 


    #define BNDInsert(nbnd, bndind, bndptr, vtx) \
      ListInsert(nbnd, bndind, bndptr, vtx)

    #define BNDDelete(nbnd, bndind, bndptr, vtx) \
      ListDelete(nbnd, bndind, bndptr, vtx)

    #define KEY_LT(a, b) ((a) > (b))

    template<typename K, typename V>
    struct pq_t {
        using PQT = kv_t<K, V>;

        size_t					nnodes;
        size_t					maxnodes;
        std::vector<PQT>		heap;
        std::vector<ssize_t>	locator;

        pq_t(size_t maxnodes)
        {
            Init(maxnodes);
        }

        void Init(size_t _maxnodes)
        {
            nnodes = 0;
            maxnodes = _maxnodes;
            heap.resize(_maxnodes);
            locator.resize(_maxnodes);
            for (size_t i = 0; i < locator.size(); ++i)
            {
                locator[i] = -1;
            }
        }

        void Reset()
        {
            ssize_t i;
            for (i = (ssize_t)nnodes - 1; i >= 0; i--)
                locator[heap[i].val] = -1;
            nnodes = 0;
        }

        size_t Length() const
        {
            return nnodes;
        }

        int Insert(V node, K key)
        {
            ssize_t i, j;
            assert(CheckHeap());
            assert(locator[node] == -1);
            i = (ssize_t)(nnodes++);
            while (i > 0) {
                j = (i - 1) >> 1;
                if (KEY_LT(key, heap[j].key)) {
                    heap[i] = heap[j];
                    locator[heap[i].val] = i;
                    i = j;
                }
                else
                    break;
            }
            assert(i >= 0);
            heap[i].key = key;
            heap[i].val = node;
            locator[node] = i;
            assert(CheckHeap());
            return 0;
        }

        int Delete(V node)
        {
            ssize_t i, j;
            K newkey, oldkey;
            assert(locator[node] != -1);
            assert(heap[locator[node]].val == node);
            assert(CheckHeap());
            i = locator[node];
            locator[node] = -1;
            if (--nnodes > 0 && heap[nnodes].val != node) {
                node = heap[nnodes].val;
                newkey = heap[nnodes].key;
                oldkey = heap[i].key;
                if (KEY_LT(newkey, oldkey)) {
                    while (i > 0) {
                        j = (i - 1) >> 1;
                        if (KEY_LT(newkey, heap[j].key)) {
                            heap[i] = heap[j];
                            locator[heap[i].val] = i;
                            i = j;
                        }
                        else
                            break;
                    }
                }
                else {
                    while ((j = (i << 1) + 1) < (ssize_t)nnodes) {
                        if (KEY_LT(heap[j].key, newkey)) {
                            if (j + 1 < (ssize_t)nnodes && KEY_LT(heap[j + 1].key, heap[j].key))
                                j++;
                            heap[i] = heap[j];
                            locator[heap[i].val] = i;
                            i = j;
                        }
                        else if (j + 1 < (ssize_t)nnodes && KEY_LT(heap[j + 1].key, newkey)) {
                            j++;
                            heap[i] = heap[j];
                            locator[heap[i].val] = i;
                            i = j;
                        }
                        else
                            break;
                    }
                }
                heap[i].key = newkey;
                heap[i].val = node;
                locator[node] = i;
            }
            assert(CheckHeap());
            return 0;
        }

        void Update(V node, K newkey)
        {
            ssize_t i, j;
            K oldkey;
            oldkey = heap[locator[node]].key;
            if (!KEY_LT(newkey, oldkey) && !KEY_LT(oldkey, newkey))
                return;
            assert(locator[node] != -1);
            assert(heap[locator[node]].val == node);
            assert(CheckHeap());
            i = locator[node];
            if (KEY_LT(newkey, oldkey)) {
                while (i > 0) {
                    j = (i - 1) >> 1;
                    if (KEY_LT(newkey, heap[j].key)) {
                        heap[i] = heap[j];
                        locator[heap[i].val] = i;
                        i = j;
                    }
                    else
                        break;
                }
            }
            else {
                while ((j = (i << 1) + 1) < (ssize_t)nnodes) {
                    if (KEY_LT(heap[j].key, newkey)) {
                        if (j + 1 < (ssize_t)nnodes && KEY_LT(heap[j + 1].key, heap[j].key))
                            j++;
                        heap[i] = heap[j];
                        locator[heap[i].val] = i;
                        i = j;
                    }
                    else if (j + 1 < (ssize_t)nnodes && KEY_LT(heap[j + 1].key, newkey)) {
                        j++;
                        heap[i] = heap[j];
                        locator[heap[i].val] = i;
                        i = j;
                    }
                    else
                        break;
                }
            }
            heap[i].key = newkey;
            heap[i].val = node;
            locator[node] = i;
            assert(CheckHeap());
            return;
        }

        V GetTop()
        {
            ssize_t i, j;
            V vtx, node;
            K key;
            assert(CheckHeap());

            if (nnodes == 0)
                return -1;

            nnodes--;
            vtx = heap[0].val;
            locator[vtx] = -1;

            if ((i = (ssize_t)nnodes) > 0) {
                key = heap[i].key;
                node = heap[i].val;
                i = 0;
                while ((j = 2 * i + 1) < (ssize_t)nnodes) {
                    if (KEY_LT(heap[j].key, key)) {
                        if (j + 1 < (ssize_t)nnodes && KEY_LT(heap[j + 1].key, heap[j].key))
                            j = j + 1;
                        heap[i] = heap[j];
                        locator[heap[i].val] = i;
                        i = j;
                    }
                    else if (j + 1 < (ssize_t)nnodes && KEY_LT(heap[j + 1].key, key)) {
                        j = j + 1;
                        heap[i] = heap[j];
                        locator[heap[i].val] = i;
                        i = j;
                    }
                    else
                        break;
                }
                heap[i].key = key;
                heap[i].val = node;
                locator[node] = i;
            }

            assert(CheckHeap());
            return vtx;
        }

        V SeeTopVal()
        {
            return (nnodes == 0 ? -1 : heap[0].val);
        }

        K SeeTopKey()
        {
            return (nnodes == 0 ? std::numeric_limits<K>::max() : heap[0].key);
        }

        int CheckHeap()
        {
            ssize_t i, j;
            if (nnodes == 0)
                return 1;

            assert(locator[heap[0].val] == 0);
            for (i = 1; i < nnodes; i++) {
                assert(locator[heap[i].val] == i);
                assert(!KEY_LT(heap[i].key, heap[(i - 1) / 2].key));
            }
            for (i = 1; i < nnodes; i++)
                assert(!KEY_LT(heap[i].key, heap[0].key));

            for (j = i = 0; i < maxnodes; i++) {
                if (locator[i] != -1)
                    j++;
            }
            assert(j == nnodes);

            return 1;
        }
    };

    typedef pq_t<int, idx_t> ipq_t;
    typedef pq_t<float, idx_t> rpq_t;

    #define rpqCreate(_x)			new rpq_t(_x)
    #define rpqReset(_q)			(_q)->Reset()
    #define rpqInsert(_q, _v, _k)	(_q)->Insert(_v, _k)
    #define rpqUpdate(_q, _v, _k)	(_q)->Update(_v, _k)
    #define rpqDelete(_q, _v)		(_q)->Delete(_v)
    #define rpqGetTop(_q)			(_q)->GetTop()
    #define rpqSeeTopKey(_q)		(_q)->SeeTopKey()
    #define rpqSeeTopVal(_q)		(_q)->SeeTopVal()
    #define rpqLength(_q)			(_q)->Length()
    #define rpqDestroy(_q)			delete (_q);

    #define ipqCreate(_x)			new ipq_t(_x)
    #define ipqReset(_q)			(_q)->Reset()
    #define ipqInsert(_q, _v, _k)	(_q)->Insert(_v, _k)
    #define ipqUpdate(_q, _v, _k)	(_q)->Update(_v, _k)
    #define ipqDelete(_q, _v)		(_q)->Delete(_v)
    #define ipqGetTop(_q)			(_q)->GetTop()
    #define ipqSeeTopKey(_q)		(_q)->SeeTopKey()
    #define ipqSeeTopVal(_q)		(_q)->SeeTopVal()
    #define ipqLength(_q)			(_q)->Length()
    #define ipqDestroy(_q)			delete (_q);

    typedef struct gk_mop_t {
        int type;
        ssize_t nbytes;
        void* ptr;
    } gk_mop_t;

    /*************************************************************************/
    /*! The following structure defines the mcore for GKlib's customized
        memory allocations. */
        /*************************************************************************/
    typedef struct gk_mcore_t {
        /* Workspace information */
        size_t coresize;     /*!< The amount of core memory that has been allocated */
        size_t corecpos;     /*!< Index of the first free location in core */
        void* core;	       /*!< Pointer to the core itself */

        /* These are for implementing a stack-based allocation scheme using both
           core and also dynamically allocated memory */
        size_t nmops;         /*!< The number of maop_t entries that have been allocated */
        size_t cmop;          /*!< Index of the first free location in maops */
        gk_mop_t* mops;       /*!< The array recording the maop_t operations */

        /* These are for keeping various statistics for wspacemalloc */
        size_t num_callocs;   /*!< The number of core mallocs */
        size_t num_hallocs;   /*!< The number of heap mallocs */
        size_t size_callocs;  /*!< The total # of bytes in core mallocs */
        size_t size_hallocs;  /*!< The total # of bytes in heap mallocs */
        size_t cur_callocs;   /*!< The current # of bytes in core mallocs */
        size_t cur_hallocs;   /*!< The current # of bytes in heap mallocs */
        size_t max_callocs;   /*!< The maximum # of bytes in core mallocs at any given time */
        size_t max_hallocs;   /*!< The maximum # of bytes in heap mallocs at any given time */

    } gk_mcore_t;

    /*************************************************************************/
    /*! This data structure stores cut-based k-way refinement info about an
        adjacent subdomain for a given vertex. */
        /*************************************************************************/
    typedef struct cnbr_t {
        idx_t pid;            /*!< The partition ID */
        idx_t ed;             /*!< The sum of the weights of the adjacent edges
                                   that are incident on pid */
    } cnbr_t;

    /*************************************************************************/
    /*! The following data structure stores holds information on degrees for k-way
        partition */
        /*************************************************************************/
    typedef struct ckrinfo_t {
        idx_t id;              /*!< The internal degree of a vertex (sum of weights) */
        idx_t ed;            	/*!< The total external degree of a vertex */
        idx_t nnbrs;          	/*!< The number of neighboring subdomains */
        idx_t inbr;            /*!< The index in the cnbr_t array where the nnbrs list
                                    of neighbors is stored */
    } ckrinfo_t;

    /*************************************************************************/
    /*! This data structure stores volume-based k-way refinement info about an
        adjacent subdomain for a given vertex. */
        /*************************************************************************/
    typedef struct vnbr_t {
        idx_t pid;            /*!< The partition ID */
        idx_t ned;            /*!< The number of the adjacent edges
                                   that are incident on pid */
        idx_t gv;             /*!< The gain in volume achieved by moving the
                                   vertex to pid */
    } vnbr_t;

    /*************************************************************************/
    /*! The following data structure holds information on degrees for k-way
        vol-based partition */
        /*************************************************************************/
    typedef struct vkrinfo_t {
        idx_t nid;             /*!< The internal degree of a vertex (count of edges) */
        idx_t ned;            	/*!< The total external degree of a vertex (count of edges) */
        idx_t gv;            	/*!< The volume gain of moving that vertex */
        idx_t nnbrs;          	/*!< The number of neighboring subdomains */
        idx_t inbr;            /*!< The index in the vnbr_t array where the nnbrs list
                                    of neighbors is stored */
    } vkrinfo_t;

    typedef struct nrinfo_t {
        idx_t edegrees[2];
    } nrinfo_t;

    typedef struct graph_t {
        idx_t nvtxs, nedges;	/* The # of vertices and edges in the graph */
        idx_t ncon;		/* The # of constrains */
        std::vector<idx_t> xadj;		/* Pointers to the locally stored vertices */
        std::vector<idx_t> vwgt;		/* Vertex weights */
        std::vector<idx_t> vsize;		/* Vertex sizes for min-volume formulation */
        std::vector<idx_t> adjncy;        /* Array that stores the adjacency lists of nvtxs */
        std::vector<idx_t> adjwgt;        /* Array that stores the weights of the adjacency lists */
        std::vector<idx_t> tvwgt;         /* The sum of the vertex weights in the graph */
        std::vector<float> invtvwgt;     /* The inverse of the sum of the vertex weights in the graph */

        std::vector<idx_t> cmap;  /* The contraction/coarsening map */

        std::vector<idx_t> label; /* The labels of the vertices for recursive bisection (pmetis/ometis) */

        /* Partition parameters */
        idx_t mincut, minvol;
        std::vector<idx_t> where, pwgts;
        idx_t nbnd;
        std::vector<idx_t> bndptr, bndind;

        /* Bisection refinement parameters */
        std::vector<idx_t> id, ed;

        /* K-way refinement parameters */
        std::vector<ckrinfo_t> ckrinfo;   /*!< The per-vertex cut-based refinement info */
        std::vector<vkrinfo_t> vkrinfo;   /*!< The per-vertex volume-based refinement info */

        /* Node refinement information */
        std::vector<nrinfo_t> nrinfo;

        /* keep track of the dropped edgewgt */
        idx_t droppedewgt;

        /* the linked-list structure of the sequence of graphs */
        struct graph_t* coarser, * finer;

    } graph_t;

    typedef struct ctrl_t {
        moptype_et  optype;	        /* Type of operation */
        mobjtype_et objtype;          /* Type of refinement objective */
        mdbglvl_et  dbglvl;		/* Controls the debugging output of the program */
        mctype_et   ctype;		/* The type of coarsening */
        miptype_et  iptype;		/* The type of initial partitioning */

        idx_t CoarsenTo;		/* The # of vertices in the coarsest graph */
        idx_t nIparts;                /* The number of initial partitions to compute */
        idx_t no2hop;                 /* Indicates if 2-hop matching will be used */
        idx_t minconn;                /* Indicates if the subdomain connectivity will be minimized */
        idx_t contig;                 /* Indicates if contiguous partitions are required */
        idx_t nseps;			/* The number of separators to be found during multiple bisections */
        idx_t ufactor;                /* The user-supplied load imbalance factor */
        idx_t compress;               /* If the graph will be compressed prior to ordering */
        idx_t ccorder;                /* If connected components will be ordered separately */
        idx_t seed;                   /* The seed for the random number generator */
        idx_t ncuts;                  /* The number of different partitionings to compute */
        idx_t niter;                  /* The number of iterations during each refinement */
        idx_t numflag;                /* The user-supplied numflag for the graph */
        idx_t dropedges;              /* Indicates if edges will be randomly dropped during coarsening */
        std::vector<idx_t> maxvwgt;	/* The maximum allowed weight for a vertex */

        idx_t ncon;                   /*!< The number of balancing constraints */
        idx_t nparts;                 /*!< The number of partitions */

        real_t pfactor;		/* .1*(user-supplied prunning factor) */

        std::vector<real_t> ubfactors;            /*!< The per-constraint ubfactors */

        std::vector<real_t> tpwgts;               /*!< The target partition weights */
        std::vector<real_t> pijbm;                /*!< The nparts*ncon multiplies for the ith partition
                                           and jth constraint for obtaining the balance */

        real_t cfactor;               /*!< The achieved compression factor */

                                   /* These are for use by the k-way refinement routines */
        size_t nbrpoolsize_max;  /*!< The maximum number of {c,v}nbr_t entries that will ever be allocated */
        size_t nbrpoolsize;      /*!< The number of {c,v}nbr_t entries that have been allocated */
        size_t nbrpoolcpos;      /*!< The position of the first free entry in the array */
        size_t nbrpoolreallocs;  /*!< The number of times the pool was resized */

        std::vector<cnbr_t> cnbrpool;     /*!< The pool of cnbr_t entries to be used during refinement.
                                   The size and current position of the pool is controlled
                                   by nnbrs & cnbrs */
        std::vector<vnbr_t> vnbrpool;     /*!< The pool of vnbr_t entries to be used during refinement.
                                   The size and current position of the pool is controlled
                                   by nnbrs & cnbrs */

                                   /* The subdomain graph, in sparse format  */
        std::vector<idx_t> maxnads;               /* The maximum allocated number of adjacent domains */
        std::vector<idx_t> nads;                  /* The number of adjacent domains */
        std::vector<std::vector<idx_t>> adids;    /* The IDs of the adjacent domains */
        std::vector<std::vector<idx_t>> adwgts;   /* The edge-weight to the adjacent domains */
        std::vector<idx_t> pvec1, pvec2;          /* Auxiliary nparts-size vectors for efficiency */
    } ctrl_t;


    size_t iargmax(size_t n, idx_t* x, size_t incx)
    {
        size_t i, j, m = 0;
        for (i = 1, j = incx; i < n; i++, j += incx)
            m = (x[j] > x[m] ? j : m);
        return (size_t)(m / incx);
    }

    idx_t imax(size_t n, idx_t* x, size_t incx)
    {
        size_t i;
        idx_t m;
        if (n <= 0) return (idx_t)0;
        for (m = (*x), x += incx, i = 1; i < n; i++, x += incx)
            m = ((*x) > m ? (*x) : m);
        return m;
    }

    idx_t iargmax_nrm(size_t n, idx_t* x, real_t* y)
    {
        idx_t m = 0;

        for (size_t i = 1; i < n; i++)
            m = (idx_t)(x[i] * y[i] > x[m] * y[m] ? i : m);

        return m;
    }

    idx_t iargmax_strd(size_t n, idx_t* x, idx_t incx)
    {
        size_t i, m = 0;

        n *= incx;
        for (i = incx; i < n; i += incx)
            m = (x[i] > x[m] ? i : m);

        return (idx_t)(m / incx);
    }

    idx_t rargmax2(size_t n, real_t* x)
    {
        size_t i, max1, max2;

        if (x[0] > x[1]) {
            max1 = 0;
            max2 = 1;
        }
        else {
            max1 = 1;
            max2 = 0;
        }

        for (i = 2; i < n; i++) {
            if (x[i] > x[max1]) {
                max2 = max1;
                max1 = i;
            }
            else if (x[i] > x[max2])
                max2 = i;
        }

        return (idx_t)max2;
    }

    idx_t iargmax2_nrm(size_t n, idx_t* x, real_t* y)
    {
        size_t i, max1, max2;

        if (x[0] * y[0] > x[1] * y[1]) {
            max1 = 0;
            max2 = 1;
        }
        else {
            max1 = 1;
            max2 = 0;
        }

        for (i = 2; i < n; i++) {
            if (x[i] * y[i] > x[max1] * y[max1]) {
                max2 = max1;
                max1 = i;
            }
            else if (x[i] * y[i] > x[max2] * y[max2])
                max2 = i;
        }

        return (idx_t)max2;
    }

    real_t rvecmaxdiff(idx_t n, real_t* x, real_t* y)
    {
        real_t max;

        max = x[0] - y[0];

        for (n--; n > 0; n--) {
            if (max < x[n] - y[n])
                max = x[n] - y[n];
        }

        return max;
    }

    int ivecle(idx_t n, idx_t* x, idx_t* z)
    {
        for (n--; n >= 0; n--) {
            if (x[n] > z[n])
                return 0;
        }

        return  1;
    }

    int ivecge(idx_t n, idx_t* x, idx_t* z)
    {
        for (n--; n >= 0; n--) {
            if (x[n] < z[n])
                return 0;
        }

        return  1;
    }

    int ivecaxpylez(idx_t n, idx_t a, idx_t* x, idx_t* y, idx_t* z)
    {
        for (n--; n >= 0; n--) {
            if (a * x[n] + y[n] > z[n])
                return 0;
        }

        return  1;
    }

    int ivecaxpygez(idx_t n, idx_t a, idx_t* x, idx_t* y, idx_t* z)
    {
        for (n--; n >= 0; n--) {
            if (a * x[n] + y[n] < z[n])
                return 0;
        }

        return  1;
    }

    template<typename T>
    T* set(size_t size, T def_val, T* p)
    {
        for (size_t i = 0; i < size; ++i)
        {
            p[i] = def_val;
        }
        return p;
    }

    idx_t* iset(size_t size, idx_t def_val, idx_t* p)
    {
        return set<idx_t>(size, def_val, p);
    }

    template<typename T>
    T* incset(size_t size, T base_val, T* p)
    {
        for (size_t i = 0; i < size; ++i)
        {
            p[i] = base_val + (T)i;
        }
        return p;
    }

    idx_t* iincset(size_t size, idx_t def_val, idx_t* p)
    {
        return incset<idx_t>(size, def_val, p);
    }

    template<typename T>
    void copy(size_t size, const T* src, T* dst)
    {
        memcpy(dst, src, size * sizeof(T));
    }

    void icopy(size_t size, const idx_t* src, idx_t* dst)
    {
        copy<idx_t>(size, src, dst);
    }

    void rcopy(size_t size, const float* src, float* dst)
    {
        copy<real_t>(size, src, dst);
    }

    template<typename T>
    T* axpy(size_t n, T alpha, T* x, size_t incx, T* y, size_t incy)
    {
        size_t i;
        T* y_in = y;
        for (i = 0; i < n; i++, x += incx, y += incy)
            *y += alpha * (*x);
        return y_in;
    }

    idx_t* iaxpy(size_t n, idx_t alpha, idx_t* x, size_t incx, idx_t* y, size_t incy)
    {
        return axpy<idx_t>(n, alpha, x, incx, y, incy);
    }

    template<typename T>
    T _sum(size_t size, T* p, size_t incx)
    {
        T s = 0;
        for (size_t i = 0; i < size; i += incx)
        {
            s += p[i];
        }
        return s;
    }
#define isum _sum<int>
#define rsum _sum<float>

    template<typename T>
    T _max(size_t n, T* x, size_t incx)
    {
        size_t i;
        T m;
        if (n <= 0)
            return (T)0;
        for (m = (*x), x += incx, i = 1; i < n; i++, x += incx)
            m = ((*x) > m ? (*x) : m);
        return m;
    }

    template<typename T>
    T _min(size_t n, T* x, size_t incx)
    {
        size_t i;
        T m;
        if (n <= 0)
            return (T)0;
        for (m = (*x), x += incx, i = 1; i < n; i++, x += incx)
            m = ((*x) < m ? (*x) : m);
        return m;
    }

#define gk_max(a, b) ((a) >= (b) ? (a) : (b))
#define gk_min(a, b) ((a) >= (b) ? (b) : (a))
#define gk_max3(a, b, c) ((a) >= (b) && (a) >= (c) ? (a) : ((b) >= (a) && (b) >= (c) ? (b) : (c)))
#define gk_SWAP(a, b, tmp) do {(tmp) = (a); (a) = (b); (b) = (tmp);} while(0) 
#define INC_DEC(a, b, val) do {(a) += (val); (b) -= (val);} while(0)
#define sign(a, b) ((a >= 0 ? b : -b))

    int gk_log2(int a)
    {
        size_t i;

        for (i = 1; a > 1; i++, a = a >> 1);
        return (int)(i - 1);
    }

    template<typename T>
    T* _scale(size_t n, T alpha, T* x, size_t incx)
    {
        size_t i;
        for (i = 0; i < n; i++, x += incx)
            (*x) *= alpha;
        return x;
    }

#define rscale _scale<float>

    idx_t ComputeVolume(graph_t* graph, idx_t* w)
    {
        idx_t i, j, k, nvtxs, nparts, totalv;
        idx_t* xadj, * adjncy, * vsize;


        nvtxs = graph->nvtxs;
        xadj = graph->xadj.data();
        adjncy = graph->adjncy.data();
        vsize = graph->vsize.data();

        nparts = w[iargmax(nvtxs, w, 1)] + 1;
        std::vector<idx_t> marker(nparts, -1);

        totalv = 0;

        for (i = 0; i < nvtxs; i++) {
            marker[w[i]] = i;
            for (j = xadj[i]; j < xadj[i + 1]; j++) {
                k = w[adjncy[j]];
                if (marker[k] != i) {
                    marker[k] = i;
                    totalv += (vsize ? vsize[i] : 1);
                }
            }
        }

        return totalv;
    }

    int BetterVBalance(idx_t ncon, real_t* invtvwgt, idx_t* v_vwgt, idx_t* u1_vwgt,
        idx_t* u2_vwgt)
    {
        idx_t i;
        real_t sum1 = 0.0, sum2 = 0.0, diff1 = 0.0, diff2 = 0.0;

        for (i = 0; i < ncon; i++) {
            sum1 += (v_vwgt[i] + u1_vwgt[i]) * invtvwgt[i];
            sum2 += (v_vwgt[i] + u2_vwgt[i]) * invtvwgt[i];
        }
        sum1 = sum1 / ncon;
        sum2 = sum2 / ncon;

        for (i = 0; i < ncon; i++) {
            diff1 += rabs(sum1 - (v_vwgt[i] + u1_vwgt[i]) * invtvwgt[i]);
            diff2 += rabs(sum2 - (v_vwgt[i] + u2_vwgt[i]) * invtvwgt[i]);
        }

        return (diff1 - diff2 >= 0);
    }

    /*************************************************************************/
    /*! This function takes two ubfactor-centered load imbalance vectors x & y,
        and returns true if y is better balanced than x. */
        /*************************************************************************/
    int BetterBalance2Way(idx_t n, real_t* x, real_t* y)
    {
        real_t nrm1 = 0.0, nrm2 = 0.0;

        for (--n; n >= 0; n--) {
            if (x[n] > 0) nrm1 += x[n] * x[n];
            if (y[n] > 0) nrm2 += y[n] * y[n];
        }
        return nrm2 < nrm1;
    }


    /*************************************************************************/
    /*! Given a vertex and two weights, this function returns 1, if the second
        partition will be more balanced than the first after the weighted
        additional of that vertex.
        The balance determination takes into account the ideal target weights
        of the two partitions.
    */
    /*************************************************************************/
    int BetterBalanceKWay(idx_t ncon, idx_t* vwgt, real_t* ubvec,
        idx_t a1, idx_t* pt1, real_t* bm1,
        idx_t a2, idx_t* pt2, real_t* bm2)
    {
        idx_t i;
        real_t tmp, nrm1 = 0.0, nrm2 = 0.0, max1 = 0.0, max2 = 0.0;

        for (i = 0; i < ncon; i++) {
            tmp = bm1[i] * (pt1[i] + a1 * vwgt[i]) - ubvec[i];
            //printf("BB: %d %+.4f ", (int)i, (float)tmp);
            nrm1 += tmp * tmp;
            max1 = (tmp > max1 ? tmp : max1);

            tmp = bm2[i] * (pt2[i] + a2 * vwgt[i]) - ubvec[i];
            //printf("%+.4f ", (float)tmp);
            nrm2 += tmp * tmp;
            max2 = (tmp > max2 ? tmp : max2);

        }
        //printf("   %.3f %.3f %.3f %.3f\n", (float)max1, (float)nrm1, (float)max2, (float)nrm2);

        if (max2 < max1)
            return 1;

        if (max2 == max1 && nrm2 < nrm1)
            return 1;

        return 0;
    }


    /*************************************************************************/
    /*! Computes the maximum load imbalance of a partitioning solution over
        all the constraints. */
        /**************************************************************************/
    real_t ComputeLoadImbalance(graph_t* graph, idx_t nparts, real_t* pijbm)
    {
        idx_t i, j, ncon, * pwgts;
        real_t max, cur;

        ncon = graph->ncon;
        pwgts = graph->pwgts.data();

        max = 1.0;
        for (i = 0; i < ncon; i++) {
            for (j = 0; j < nparts; j++) {
                cur = pwgts[j * ncon + i] * pijbm[j * ncon + i];
                if (cur > max)
                    max = cur;
            }
        }

        return max;
    }


    /*************************************************************************/
    /*! Computes the maximum load imbalance difference of a partitioning
        solution over all the constraints.
        The difference is defined with respect to the allowed maximum
        unbalance for the respective constraint.
     */
     /**************************************************************************/
    real_t ComputeLoadImbalanceDiff(graph_t* graph, idx_t nparts, real_t* pijbm,
        real_t* ubvec)
    {
        idx_t i, j, ncon, * pwgts;
        real_t max, cur;

        ncon = graph->ncon;
        pwgts = graph->pwgts.data();

        max = -1.0;
        for (i = 0; i < ncon; i++) {
            for (j = 0; j < nparts; j++) {
                cur = pwgts[j * ncon + i] * pijbm[j * ncon + i] - ubvec[i];
                if (cur > max)
                    max = cur;
            }
        }

        return max;
    }


    /*************************************************************************/
    /*! Computes the difference between load imbalance of each constraint across
        the partitions minus the desired upper bound on the load imabalnce.
        It also returns the maximum load imbalance across the partitions &
        constraints. */
        /**************************************************************************/
    real_t ComputeLoadImbalanceDiffVec(graph_t* graph, idx_t nparts, real_t* pijbm,
        real_t* ubfactors, real_t* diffvec)
    {
        idx_t i, j, ncon, * pwgts;
        real_t cur, max;

        ncon = graph->ncon;
        pwgts = graph->pwgts.data();

        for (max = -1.0, i = 0; i < ncon; i++) {
            diffvec[i] = pwgts[i] * pijbm[i] - ubfactors[i];
            for (j = 1; j < nparts; j++) {
                cur = pwgts[j * ncon + i] * pijbm[j * ncon + i] - ubfactors[i];
                if (cur > diffvec[i])
                    diffvec[i] = cur;
            }
            if (max < diffvec[i])
                max = diffvec[i];
        }

        return max;
    }

    int IsBalanced(ctrl_t* ctrl, graph_t* graph, real_t ffactor)
    {
        return
            (ComputeLoadImbalanceDiff(graph, ctrl->nparts, ctrl->pijbm.data(), ctrl->ubfactors.data())
                <= ffactor);
    }

    /*************************************************************************/
    /*! Computes the load imbalance of each constraint across the partitions. */
    /**************************************************************************/
    void ComputeLoadImbalanceVec(graph_t* graph, idx_t nparts, real_t* pijbm,
        real_t* lbvec)
    {
        idx_t i, j, ncon, * pwgts;
        real_t cur;

        ncon = graph->ncon;
        pwgts = graph->pwgts.data();

        for (i = 0; i < ncon; i++) {
            lbvec[i] = pwgts[i] * pijbm[i];
            for (j = 1; j < nparts; j++) {
                cur = pwgts[j * ncon + i] * pijbm[j * ncon + i];
                if (cur > lbvec[i])
                    lbvec[i] = cur;
            }
        }
    }

    idx_t ComputeCut(graph_t* graph, idx_t* where)
    {
        idx_t i, j, cut;

        if (graph->adjwgt.empty()) {
            for (cut = 0, i = 0; i < graph->nvtxs; i++) {
                for (j = graph->xadj[i]; j < graph->xadj[i + 1]; j++)
                    if (where[i] != where[graph->adjncy[j]])
                        cut++;
            }
        }
        else {
            for (cut = 0, i = 0; i < graph->nvtxs; i++) {
                for (j = graph->xadj[i]; j < graph->xadj[i + 1]; j++)
                    if (where[i] != where[graph->adjncy[j]])
                        cut += graph->adjwgt[j];
            }
        }

        return cut / 2;
    }

    idx_t IsArticulationNode(idx_t i, idx_t* xadj, idx_t* adjncy, idx_t* where,
        idx_t* bfslvl, idx_t* bfsind, idx_t* bfsmrk)
    {
        idx_t ii, j, k = 0, head, tail, nhits, tnhits, from, BFSDEPTH = 5;

        from = where[i];

        /* Determine if the vertex is safe to move from a contiguity standpoint */
        for (tnhits = 0, j = xadj[i]; j < xadj[i + 1]; j++) {
            if (where[adjncy[j]] == from) {
                ASSERT(bfsmrk[adjncy[j]] == 0);
                ASSERT(bfslvl[adjncy[j]] == 0);
                bfsmrk[k = adjncy[j]] = 1;
                tnhits++;
            }
        }

        /* Easy cases */
        if (tnhits == 0)
            return 0;
        if (tnhits == 1) {
            bfsmrk[k] = 0;
            return 0;
        }

        ASSERT(bfslvl[i] == 0);
        bfslvl[i] = 1;

        bfsind[0] = k; /* That was the last one from the previous loop */
        bfslvl[k] = 1;
        bfsmrk[k] = 0;
        head = 0;
        tail = 1;

        /* Do a limited BFS traversal to see if you can get to all the other nodes */
        for (nhits = 1; head < tail; ) {
            ii = bfsind[head++];
            for (j = xadj[ii]; j < xadj[ii + 1]; j++) {
                if (where[k = adjncy[j]] == from) {
                    if (bfsmrk[k]) {
                        bfsmrk[k] = 0;
                        if (++nhits == tnhits)
                            break;
                    }
                    if (bfslvl[k] == 0 && bfslvl[ii] < BFSDEPTH) {
                        bfsind[tail++] = k;
                        bfslvl[k] = bfslvl[ii] + 1;
                    }
                }
            }
            if (nhits == tnhits)
                break;
        }

        /* Reset the various BFS related arrays */
        bfslvl[i] = 0;
        for (j = 0; j < tail; j++)
            bfslvl[bfsind[j]] = 0;


        /* Reset the bfsmrk array for the next vertex when has not already being cleared */
        if (nhits < tnhits) {
            for (j = xadj[i]; j < xadj[i + 1]; j++)
                if (where[adjncy[j]] == from)
                    bfsmrk[adjncy[j]] = 0;
        }

        return (nhits != tnhits);
    }

    /*************************************************************************
    * These macros deal with id/ed updating during k-way refinement
    **************************************************************************/
#define UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, \
            nbnd, bndptr, bndind, bndtype) \
   do { \
     where[i] = to; \
     myrinfo->ed += myrinfo->id-mynbrs[k].ed; \
     SWAP(myrinfo->id, mynbrs[k].ed, j); \
     if (mynbrs[k].ed == 0) \
       mynbrs[k] = mynbrs[--myrinfo->nnbrs]; \
     else \
       mynbrs[k].pid = from; \
     \
     /* Update the boundary information. Both deletion and addition is \
        allowed as this routine can be used for moving arbitrary nodes. */ \
     if (bndtype == BNDTYPE_REFINE) { \
       if (bndptr[i] != -1 && myrinfo->ed - myrinfo->id < 0) \
         BNDDelete(nbnd, bndind, bndptr, i); \
       if (bndptr[i] == -1 && myrinfo->ed - myrinfo->id >= 0) \
         BNDInsert(nbnd, bndind, bndptr, i); \
     } \
     else { \
       if (bndptr[i] != -1 && myrinfo->ed <= 0) \
         BNDDelete(nbnd, bndind, bndptr, i); \
       if (bndptr[i] == -1 && myrinfo->ed > 0) \
         BNDInsert(nbnd, bndind, bndptr, i); \
     } \
   } while(0) 


#define UpdateAdjacentVertexInfoAndBND(ctrl, vid, adjlen, me, from, to, \
            myrinfo, ewgt, nbnd, bndptr, bndind, bndtype) \
   do { \
     idx_t k; \
     cnbr_t *mynbrs; \
     \
     if (myrinfo->inbr == -1) { \
       myrinfo->inbr  = cnbrpoolGetNext(ctrl, adjlen); \
       myrinfo->nnbrs = 0; \
     } \
     ASSERT(CheckRInfo(ctrl, myrinfo)); \
     \
     mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr; \
     \
     /* Update global ID/ED and boundary */ \
     if (me == from) { \
       INC_DEC(myrinfo->ed, myrinfo->id, (ewgt)); \
       if (bndtype == BNDTYPE_REFINE) { \
         if (myrinfo->ed-myrinfo->id >= 0 && bndptr[(vid)] == -1) \
           BNDInsert(nbnd, bndind, bndptr, (vid)); \
       } \
       else { \
         if (myrinfo->ed > 0 && bndptr[(vid)] == -1) \
           BNDInsert(nbnd, bndind, bndptr, (vid)); \
       } \
     } \
     else if (me == to) { \
       INC_DEC(myrinfo->id, myrinfo->ed, (ewgt)); \
       if (bndtype == BNDTYPE_REFINE) { \
         if (myrinfo->ed-myrinfo->id < 0 && bndptr[(vid)] != -1) \
           BNDDelete(nbnd, bndind, bndptr, (vid)); \
       } \
       else { \
         if (myrinfo->ed <= 0 && bndptr[(vid)] != -1) \
           BNDDelete(nbnd, bndind, bndptr, (vid)); \
       } \
     } \
     \
     /* Remove contribution from the .ed of 'from' */ \
     if (me != from) { \
       for (k=0; k<myrinfo->nnbrs; k++) { \
         if (mynbrs[k].pid == from) { \
           if (mynbrs[k].ed == (ewgt)) \
             mynbrs[k] = mynbrs[--myrinfo->nnbrs]; \
           else \
             mynbrs[k].ed -= (ewgt); \
           break; \
         } \
       } \
     } \
     \
     /* Add contribution to the .ed of 'to' */ \
     if (me != to) { \
       for (k=0; k<myrinfo->nnbrs; k++) { \
         if (mynbrs[k].pid == to) { \
           mynbrs[k].ed += (ewgt); \
           break; \
         } \
       } \
       if (k == myrinfo->nnbrs) { \
         mynbrs[k].pid  = to; \
         mynbrs[k].ed   = (ewgt); \
         myrinfo->nnbrs++; \
       } \
     } \
     \
     ASSERT(CheckRInfo(ctrl, myrinfo));\
   } while(0) 


#define UpdateQueueInfo(queue, vstatus, vid, me, from, to, myrinfo, oldnnbrs, \
            nupd, updptr, updind, bndtype) \
   do { \
     real_t rgain; \
     \
     if (me == to || me == from || oldnnbrs != myrinfo->nnbrs) {  \
       rgain = (myrinfo->nnbrs > 0 ?  \
                1.0f*myrinfo->ed/sqrtf((real_t)myrinfo->nnbrs) : 0.0f) - myrinfo->id; \
   \
       if (bndtype == BNDTYPE_REFINE) { \
         if (vstatus[(vid)] == VPQSTATUS_PRESENT) { \
           if (myrinfo->ed-myrinfo->id >= 0) \
             rpqUpdate(queue, (vid), rgain); \
           else { \
             rpqDelete(queue, (vid)); \
             vstatus[(vid)] = VPQSTATUS_NOTPRESENT; \
             ListDelete(nupd, updind, updptr, (vid)); \
           } \
         } \
         else if (vstatus[(vid)] == VPQSTATUS_NOTPRESENT && myrinfo->ed-myrinfo->id >= 0) { \
           rpqInsert(queue, (vid), rgain); \
           vstatus[(vid)] = VPQSTATUS_PRESENT; \
           ListInsert(nupd, updind, updptr, (vid)); \
         } \
       } \
       else { \
         if (vstatus[(vid)] == VPQSTATUS_PRESENT) { \
           if (myrinfo->ed > 0) \
             rpqUpdate(queue, (vid), rgain); \
           else { \
             rpqDelete(queue, (vid)); \
             vstatus[(vid)] = VPQSTATUS_NOTPRESENT; \
             ListDelete(nupd, updind, updptr, (vid)); \
           } \
         } \
         else if (vstatus[(vid)] == VPQSTATUS_NOTPRESENT && myrinfo->ed > 0) { \
           rpqInsert(queue, (vid), rgain); \
           vstatus[(vid)] = VPQSTATUS_PRESENT; \
           ListInsert(nupd, updind, updptr, (vid)); \
         } \
       } \
     } \
   } while(0) 


#define SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, vtmp) \
  do { \
    idx_t j, k, l, nadd, to; \
    for (j=0; j<myrinfo->nnbrs; j++) { \
      safetos[to = mynbrs[j].pid] = 0; \
      \
      /* uncompress the connectivity info for the 'to' subdomain */ \
      for (k=0; k<nads[to]; k++) \
        vtmp[adids[to][k]] = 1; \
      \
      for (nadd=0, k=0; k<myrinfo->nnbrs; k++) { \
        if (k == j) \
          continue; \
        \
        l = mynbrs[k].pid; \
        if (vtmp[l] == 0) { \
          if (nads[l] > maxndoms-1) { \
            nadd = maxndoms; \
            break; \
          } \
          nadd++; \
        } \
      } \
      if (nads[to]+nadd <= maxndoms) \
        safetos[to] = 1; \
      if (nadd == 0) \
        safetos[to] = 2; \
      \
      /* cleanup the connectivity info due to the 'to' subdomain */ \
      for (k=0; k<nads[to]; k++) \
        vtmp[adids[to][k]] = 0; \
    } \
  } while (0)

    void SelectQueue(graph_t* graph, real_t* pijbm, real_t* ubfactors,
        rpq_t** queues, idx_t* from, idx_t* cnum)
    {
        idx_t ncon, i, part;
        real_t max, tmp;

        ncon = graph->ncon;

        *from = -1;
        *cnum = -1;

        /* First determine the side and the queue, irrespective of the presence of nodes.
           The side & queue is determined based on the most violated balancing constraint. */
        for (max = 0.0, part = 0; part < 2; part++) {
            for (i = 0; i < ncon; i++) {
                tmp = graph->pwgts[part * ncon + i] * pijbm[part * ncon + i] - ubfactors[i];
                /* the '=' in the test below is to ensure that under tight constraints
                   the partition that is at the max is selected */
                if (tmp >= max) {
                    max = tmp;
                    *from = part;
                    *cnum = i;
                }
            }
        }


        if (*from != -1) {
            /* in case the desired queue is empty, select a queue from the same side */
            if (rpqLength(queues[2 * (*cnum) + (*from)]) == 0) {
                for (i = 0; i < ncon; i++) {
                    if (rpqLength(queues[2 * i + (*from)]) > 0) {
                        max = graph->pwgts[(*from) * ncon + i] * pijbm[(*from) * ncon + i] - ubfactors[i];
                        *cnum = i;
                        break;
                    }
                }

                for (i++; i < ncon; i++) {
                    tmp = graph->pwgts[(*from) * ncon + i] * pijbm[(*from) * ncon + i] - ubfactors[i];
                    if (tmp > max && rpqLength(queues[2 * i + (*from)]) > 0) {
                        max = tmp;
                        *cnum = i;
                    }
                }
            }

            /*
            printf("Selected1 %"PRIDX"(%"PRIDX") -> %"PRIDX" [%5"PRREAL"]\n",
                *from, *cnum, rpqLength(queues[2*(*cnum)+(*from)]), max);
            */
        }
        else {
            /* the partitioning does not violate balancing constraints, in which case select
               a queue based on cut criteria */
            for (part = 0; part < 2; part++) {
                for (i = 0; i < ncon; i++) {
                    if (rpqLength(queues[2 * i + part]) > 0 &&
                        (*from == -1 || rpqSeeTopKey(queues[2 * i + part]) > max)) {
                        max = rpqSeeTopKey(queues[2 * i + part]);
                        *from = part;
                        *cnum = i;
                    }
                }
            }
            /*
            printf("Selected2 %"PRIDX"(%"PRIDX") -> %"PRIDX"\n",
                *from, *cnum, rpqLength(queues[2*(*cnum)+(*from)]), max);
            */
        }
    }

    idx_t CheckRInfo(ctrl_t* ctrl, ckrinfo_t* rinfo)
    {
        idx_t i, j;
        cnbr_t* nbrs;
        (void)nbrs;
        // ASSERT(ctrl->nbrpoolcpos >= 0);
        ASSERT(rinfo->nnbrs < ctrl->nparts);

        nbrs = ctrl->cnbrpool.data() + rinfo->inbr;

        for (i = 0; i < rinfo->nnbrs; i++) {
            for (j = i + 1; j < rinfo->nnbrs; j++)
                ASSERT(nbrs[i].pid != nbrs[j].pid);
        }

        return 1;
    }

    idx_t CheckBnd(graph_t* graph)
    {
        idx_t i, j, nvtxs, nbnd;
        idx_t* xadj, * adjncy, * where;

        nvtxs = graph->nvtxs;
        xadj = graph->xadj.data();
        adjncy = graph->adjncy.data();
        where = graph->where.data();

        for (nbnd = 0, i = 0; i < nvtxs; i++) {
            if (xadj[i + 1] - xadj[i] == 0)
                nbnd++;   /* Islands are considered to be boundary vertices */

            for (j = xadj[i]; j < xadj[i + 1]; j++) {
                if (where[i] != where[adjncy[j]]) {
                    nbnd++;
                    // assert(bndptr[i] != -1);
                    // assert(bndind[bndptr[i]] == i);
                    break;
                }
            }
        }

        assert(nbnd == graph->nbnd);

        return 1;
    }

    idx_t CheckBnd2(graph_t* graph)
    {
        idx_t i, j, nvtxs, nbnd, id, ed;
        idx_t* xadj, * adjncy, * where, * bndptr, * bndind;
        (void)bndptr; (void)bndind;
        nvtxs = graph->nvtxs;
        xadj = graph->xadj.data();
        adjncy = graph->adjncy.data();
        where = graph->where.data();
        bndptr = graph->bndptr.data();
        bndind = graph->bndind.data();

        for (nbnd = 0, i = 0; i < nvtxs; i++) {
            id = ed = 0;
            for (j = xadj[i]; j < xadj[i + 1]; j++) {
                if (where[i] != where[adjncy[j]])
                    ed += graph->adjwgt[j];
                else
                    id += graph->adjwgt[j];
            }
            if (ed - id >= 0 && xadj[i] < xadj[i + 1]) {
                nbnd++;
                ASSERT(bndptr[i] != -1);
                ASSERT(bndind[bndptr[i]] == i);
            }
        }

        ASSERT(nbnd == graph->nbnd);

        return 1;
    }

    void InitRandom(idx_t seed)
    {
        srand((seed == -1 ? 4321 : seed));
    }

    typedef size_t RNGT;

    RNGT randInRange(RNGT max)
    {
        return (RNGT)((rand()) % max);
    }
#define irandInRange(_x) (idx_t)randInRange(_x)

    template<typename VALT>
    void randArrayPermute(RNGT n, VALT* p, RNGT nshuffles, int flag)
    {
        RNGT i, u, v;
        VALT tmp;
        if (flag == 1) {
            for (i = 0; i < n; i++)
                p[i] = (VALT)i;
        }

        if (n < 10) {
            for (i = 0; i < n; i++) {
                v = randInRange(n);
                u = randInRange(n);
                gk_SWAP(p[v], p[u], tmp);
            }
        }
        else {
            for (i = 0; i < nshuffles; i++) {
                v = randInRange(n - 3);
                u = randInRange(n - 3);
                gk_SWAP(p[v + 0], p[u + 2], tmp);
                gk_SWAP(p[v + 1], p[u + 3], tmp);
                gk_SWAP(p[v + 2], p[u + 0], tmp);
                gk_SWAP(p[v + 3], p[u + 1], tmp);
            }
        }
    }

    void irandArrayPermute(RNGT n, idx_t* p, RNGT nshuffles, int flag)
    {
        randArrayPermute<int>(n, p, nshuffles, flag);
    }

    class MetisGraphPartition
    {
    public:
        MetisGraphPartition()
        {
            m_status = METIS_OK;
            m_error_hander = nullptr;
        }

        ~MetisGraphPartition()
        {
        }

        int PartGraphRecursive(idx_t* nvtxs, idx_t* ncon, idx_t* xadj,
            idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt,
            idx_t* nparts, real_t* tpwgts, real_t* ubvec, idx_t* options,
            idx_t* objval, idx_t* part)
        {
            graph_t* graph;
            ctrl_t* ctrl;
            m_status = METIS_OK;

            ctrl = SetupCtrl(METIS_OP_PMETIS, options, *ncon, *nparts, tpwgts, ubvec);
            if (!ctrl) {
                return METIS_ERROR_INPUT;
            }

            graph = SetupGraph(ctrl, *nvtxs, *ncon, xadj, adjncy, vwgt, vsize, adjwgt);

            iset(*nvtxs, 0, part);
            *objval = (*nparts == 1 ? 0 : MlevelRecursiveBisection(ctrl, graph, *nparts, part, ctrl->tpwgts.data(), 0));

            delete ctrl;

            return m_status;
        }

        int PartGraphKway(idx_t* nvtxs, idx_t* ncon, idx_t* xadj,
            idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt,
            idx_t* nparts, real_t* tpwgts, real_t* ubvec, idx_t* options,
            idx_t* objval, idx_t* part)
        {
            m_status = METIS_OK;

            ctrl_t* ctrl = SetupCtrl(METIS_OP_KMETIS, options, *ncon, *nparts, tpwgts, ubvec);
            if (!ctrl) {
                return METIS_ERROR_INPUT;
            }

            graph_t* graph = SetupGraph(ctrl, *nvtxs, *ncon, xadj, adjncy, vwgt, vsize, adjwgt);

            /* set up multipliers for making balance computations easier */
            SetupKWayBalMultipliers(ctrl, graph);

            /* set various run parameters that depend on the graph */
            ctrl->CoarsenTo = gk_max((*nvtxs) / (40 * gk_log2(*nparts)), 30 * (*nparts));
            ctrl->nIparts = (ctrl->nIparts != -1 ? ctrl->nIparts : (ctrl->CoarsenTo == 30 * (*nparts) ? 4 : 5));

            /* take care contiguity requests for disconnected graphs */
            if (ctrl->contig && !IsConnected(graph, 0))
                gk_errexit("METIS Error: A contiguous partition is requested for a non-contiguous input graph.\n");

            iset(*nvtxs, 0, part);
            if (ctrl->dbglvl & 512)
                *objval = (*nparts == 1 ? 0 : BlockKWayPartitioning(ctrl, graph, part));
            else
                *objval = (*nparts == 1 ? 0 : MlevelKWayPartitioning(ctrl, graph, part));

            delete ctrl;

            return m_status;
        }

        idx_t BlockKWayPartitioning(ctrl_t* ctrl, graph_t* graph, idx_t* part)
        {
            idx_t i, ii, j, nvtxs;
            idx_t* vwgt;
            idx_t nparts, mynparts;
            std::vector<idx_t> fpwgts, cpwgts, fpart, perm;
            ipq_t* queue;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            vwgt = graph->vwgt.data();

            nparts = ctrl->nparts;

            mynparts = gk_min(100 * nparts, (idx_t)sqrt(nvtxs));

            for (i = 0; i < nvtxs; i++)
                part[i] = i % nparts;
            irandArrayPermute(nvtxs, part, 4 * nvtxs, 0);
            printf("Random cut: %d\n", (int)ComputeCut(graph, part));

            /* create the initial multi-section */
            mynparts = GrowMultisection(ctrl, graph, mynparts, part);

            /* balance using label-propagation and refine using a randomized greedy strategy */
            BalanceAndRefineLP(ctrl, graph, mynparts, part);

            /* determine the size of the fine partitions */
            fpwgts.resize(mynparts, 0);
            for (i = 0; i < nvtxs; i++)
                fpwgts[part[i]] += vwgt[i];

            /* create and initialize the queue that will determine
               where to put the next one */
            cpwgts.resize(nparts, 0);
            queue = ipqCreate(nparts);
            for (i = 0; i < nparts; i++)
                ipqInsert(queue, i, 0);

            /* assign the fine partitions into the coarse partitions */
            fpart.resize(mynparts);
            perm.resize(mynparts);
            irandArrayPermute(mynparts, perm.data(), mynparts, 1);
            for (ii = 0; ii < mynparts; ii++) {
                i = perm[ii];
                j = ipqSeeTopVal(queue);
                fpart[i] = j;
                cpwgts[j] += fpwgts[i];
                ipqUpdate(queue, j, -cpwgts[j]);
            }
            ipqDestroy(queue);

            for (i = 0; i < nparts; i++)
                printf("cpwgts[%d] = %d\n", (int)i, (int)cpwgts[i]);

            for (i = 0; i < nvtxs; i++)
                part[i] = fpart[part[i]];

            WCOREPOP;

            return ComputeCut(graph, part);
        }

        void BalanceAndRefineLP(ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* where)
        {
            idx_t ii, i, j, k, u, v, nvtxs, iter;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt;
            idx_t tvwgt, maxpwgt, minpwgt;
            idx_t from, to, nmoves, nnbrs;
            real_t ubfactor;
            std::vector<idx_t> pwgts, nbrids, perm, nbrwgts, nbrmrks;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            pwgts.resize(nparts, 0);

            ubfactor = I2RUBFACTOR(ctrl->ufactor);
            tvwgt = isum(nvtxs, vwgt, 1);
            maxpwgt = (idx_t)((ubfactor * tvwgt) / nparts);
            minpwgt = (idx_t)((1.0f * tvwgt) / (ubfactor * nparts));

            for (i = 0; i < nvtxs; i++)
                pwgts[where[i]] += vwgt[i];

            /* for randomly visiting the vertices */
            perm.resize(nvtxs);
            iincset(perm.size(), 0, perm.data());

            /* for keeping track of adjacent partitions */
            nbrids.resize(nparts);
            nbrwgts.resize(nparts, 0);
            nbrmrks.resize(nparts, -1);

            /* perform a fixed number of balancing LP iterations */
            for (iter = 0; iter < ctrl->niter; iter++) {
                if (imax(nparts, pwgts.data(), 1) * nparts < ubfactor * tvwgt)
                    break;

                irandArrayPermute(nvtxs, perm.data(), nvtxs / 8, 1);
                nmoves = 0;

                for (ii = 0; ii < nvtxs; ii++) {
                    u = perm[ii];

                    from = where[u];
                    if (pwgts[from] - vwgt[u] < minpwgt)
                        continue;

                    nnbrs = 0;
                    for (j = xadj[u]; j < xadj[u + 1]; j++) {
                        v = adjncy[j];
                        to = where[v];

                        if (pwgts[to] + vwgt[u] > maxpwgt)
                            continue; /* skip if 'to' is overweight */

                        if ((k = nbrmrks[to]) == -1) {
                            nbrmrks[to] = k = nnbrs++;
                            nbrids[k] = to;
                        }
                        nbrwgts[k] += xadj[v + 1] - xadj[v];
                    }
                    if (nnbrs == 0)
                        continue;

                    to = nbrids[iargmax(nnbrs, nbrwgts.data(), 1)];
                    if (from != to) {
                        where[u] = to;
                        INC_DEC(pwgts[to], pwgts[from], vwgt[u]);
                        nmoves++;
                    }

                    for (k = 0; k < nnbrs; k++) {
                        nbrmrks[nbrids[k]] = -1;
                        nbrwgts[k] = 0;
                    }

                }

                if (nmoves == 0)
                    break;
            }

            /* perform a fixed number of refinement LP iterations */
            for (iter = 0; iter < ctrl->niter; iter++) {
                irandArrayPermute(nvtxs, perm.data(), nvtxs / 8, 1);
                nmoves = 0;

                for (ii = 0; ii < nvtxs; ii++) {
                    u = perm[ii];

                    from = where[u];
                    if (pwgts[from] - vwgt[u] < minpwgt)
                        continue;

                    nnbrs = 0;
                    for (j = xadj[u]; j < xadj[u + 1]; j++) {
                        v = adjncy[j];
                        to = where[v];

                        if (to != from && pwgts[to] + vwgt[u] > maxpwgt)
                            continue; /* skip if 'to' is overweight */

                        if ((k = nbrmrks[to]) == -1) {
                            nbrmrks[to] = k = nnbrs++;
                            nbrids[k] = to;
                        }
                        nbrwgts[k] += adjwgt[j];
                    }
                    if (nnbrs == 0)
                        continue;

                    to = nbrids[iargmax(nnbrs, nbrwgts.data(), 1)];
                    if (from != to) {
                        where[u] = to;
                        INC_DEC(pwgts[to], pwgts[from], vwgt[u]);
                        nmoves++;
                    }

                    for (k = 0; k < nnbrs; k++) {
                        nbrmrks[nbrids[k]] = -1;
                        nbrwgts[k] = 0;
                    }

                }

                if (nmoves == 0)
                    break;
            }

            WCOREPOP;
        }

        idx_t GrowMultisection(ctrl_t* ctrl, graph_t* graph, idx_t nparts, idx_t* where)
        {
            idx_t i, j, k, l, nvtxs, nleft, first, last;
            idx_t* xadj, * vwgt, * adjncy;
            std::vector<idx_t> queue, pwgts;
            idx_t tvwgt, maxpwgt;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vwgt = graph->xadj.data();
            adjncy = graph->adjncy.data();

            queue.resize(nvtxs);

            /* Select the seeds for the nparts-way BFS */
            for (nleft = 0, i = 0; i < nvtxs; i++) {
                if (xadj[i + 1] - xadj[i] > 1) /* a seed's degree should be > 1 */
                    where[nleft++] = i;
            }
            nparts = gk_min(nparts, nleft);
            for (i = 0; i < nparts; i++) {
                j = irandInRange(nleft);
                queue[i] = where[j];
                where[j] = --nleft;
            }

            pwgts.resize(nparts, 0);
            tvwgt = isum(nvtxs, vwgt, 1);
            maxpwgt = (idx_t)((1.5 * tvwgt) / nparts);

            iset(nvtxs, -1, where);
            for (i = 0; i < nparts; i++) {
                where[queue[i]] = i;
                pwgts[i] = vwgt[queue[i]];
            }

            first = 0;
            last = nparts;
            nleft = nvtxs - nparts;


            /* Start the BFS from queue to get a partition */
            while (first < last) {
                i = queue[first++];
                l = where[i];
                if (pwgts[l] > maxpwgt)
                    continue;

                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                    k = adjncy[j];
                    if (where[k] == -1) {
                        if (pwgts[l] + vwgt[k] > maxpwgt)
                            break;
                        pwgts[l] += vwgt[k];
                        where[k] = l;
                        queue[last++] = k;
                        nleft--;
                    }
                }
            }

            /* Assign the unassigned vertices randomly to the nparts partitions */
            if (nleft > 0) {
                for (i = 0; i < nvtxs; i++) {
                    if (where[i] == -1)
                        where[i] = irandInRange(nparts);
                }
            }

            WCOREPOP;

            return nparts;
        }

        void AllocateRefinementWorkSpace(ctrl_t* ctrl, idx_t nbrpoolsize_max, idx_t nbrpoolsize)
        {
            ctrl->nbrpoolsize_max = nbrpoolsize_max;
            ctrl->nbrpoolsize = nbrpoolsize;
            ctrl->nbrpoolcpos = 0;
            ctrl->nbrpoolreallocs = 0;

            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
                ctrl->cnbrpool.resize(ctrl->nbrpoolsize);
                break;

            case METIS_OBJTYPE_VOL:
                ctrl->vnbrpool.resize(ctrl->nbrpoolsize);
                break;

            default:
                gk_errexit("Unknown objtype of \n");
            }

            if (ctrl->minconn) {
                ctrl->pvec1.resize(ctrl->nparts + 1);
                ctrl->pvec2.resize(ctrl->nparts + 1);
                ctrl->maxnads.resize(ctrl->nparts, INIT_MAXNAD);
                ctrl->nads.resize(ctrl->nparts);

                ctrl->adids.resize(ctrl->nparts);
                for (size_t i = 0; i < ctrl->adids.size(); ++i)
                {
                    ctrl->adids[i].resize(INIT_MAXNAD, 0);
                }

                ctrl->adwgts.resize(ctrl->nparts);
                for (size_t i = 0; i < ctrl->adwgts.size(); ++i)
                {
                    ctrl->adwgts[i].resize(INIT_MAXNAD, 0);
                }
            }
        }

        idx_t MlevelKWayPartitioning(ctrl_t* ctrl, graph_t* graph, idx_t* part)
        {
            idx_t i, curobj = 0, bestobj = 0;
            real_t curbal = 0.0, bestbal = 0.0;
            graph_t* cgraph;

            for (i = 0; i < ctrl->ncuts; i++) {
                cgraph = CoarsenGraph(ctrl, graph);

                AllocateKWayPartitionMemory(ctrl, cgraph);

                /* Compute the initial partitioning */
                InitKWayPartitioning(ctrl, cgraph);

                AllocateRefinementWorkSpace(ctrl, graph->nedges, 2 * cgraph->nedges);

                RefineKWay(ctrl, graph, cgraph);

                switch (ctrl->objtype) {
                case METIS_OBJTYPE_CUT:
                    curobj = graph->mincut;
                    break;

                case METIS_OBJTYPE_VOL:
                    curobj = graph->minvol;
                    break;

                default:
                    gk_errexit("Unknown objtype: %d\n");
                }

                curbal = ComputeLoadImbalanceDiff(graph, ctrl->nparts, ctrl->pijbm.data(), ctrl->ubfactors.data());

                if (i == 0
                    || (curbal <= 0.0005 && bestobj > curobj)
                    || (bestbal > 0.0005 && curbal < bestbal)) {
                    icopy(graph->nvtxs, graph->where.data(), part);
                    bestobj = curobj;
                    bestbal = curbal;
                }

                FreeRData(graph);

                if (bestobj == 0)
                    break;
            }

            delete graph;

            return bestobj;
        }

        void RefineKWay(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph)
        {
            idx_t i, nlevels, contig = ctrl->contig;
            graph_t* ptr;
            // Determine how many levels are there
            for (ptr = graph, nlevels = 0; ptr != orggraph; ptr = ptr->finer, nlevels++);

            // Compute the parameters of the coarsest graph
            ComputeKWayPartitionParams(ctrl, graph);

            // Try to minimize the sub-domain connectivity
            if (ctrl->minconn)
                EliminateSubDomainEdges(ctrl, graph);

            // Deal with contiguity constraints at the beginning
            if (contig && FindPartitionInducedComponents(graph, graph->where.data(), NULL, NULL) > ctrl->nparts) {
                EliminateComponents(ctrl, graph);

                ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
                Greedy_KWayOptimize(ctrl, graph, 5, 0, OMODE_BALANCE);

                ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
                Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE);

                ctrl->contig = 0;
            }

            // Refine each successively finer graph
            for (i = 0; ; i++) {
                if (ctrl->minconn && i == nlevels / 2)
                    EliminateSubDomainEdges(ctrl, graph);

                if (2 * i >= nlevels && !IsBalanced(ctrl, graph, 0.02f)) {
                    ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
                    Greedy_KWayOptimize(ctrl, graph, 1, 0, OMODE_BALANCE);
                    ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
                }

                Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 5.0, OMODE_REFINE);

                // Deal with contiguity constraints in the middle
                if (contig && i == nlevels / 2) {
                    if (FindPartitionInducedComponents(graph, graph->where.data(), NULL, NULL) > ctrl->nparts) {
                        EliminateComponents(ctrl, graph);

                        if (!IsBalanced(ctrl, graph, 0.02f)) {
                            ctrl->contig = 1;
                            ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
                            Greedy_KWayOptimize(ctrl, graph, 5, 0, OMODE_BALANCE);

                            ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
                            Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE);
                            ctrl->contig = 0;
                        }
                    }
                }

                if (graph == orggraph)
                    break;

                graph = graph->finer;

                // graph_ReadFromDisk(ctrl, graph);

                ASSERT(graph->vwgt.size() > 0);

                ProjectKWayPartition(ctrl, graph);
            }

            // Deal with contiguity requirement at the end
            ctrl->contig = contig;
            if (contig && FindPartitionInducedComponents(graph, graph->where.data(), NULL, NULL) > ctrl->nparts)
                EliminateComponents(ctrl, graph);

            if (!IsBalanced(ctrl, graph, 0.0)) {
                ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
                Greedy_KWayOptimize(ctrl, graph, 10, 0, OMODE_BALANCE);

                ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
                Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE);
            }

            if (ctrl->contig)
                ASSERT(FindPartitionInducedComponents(graph, graph->where.data(), NULL, NULL) == ctrl->nparts);
        }

        void ComputeKWayPartitionParams(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, j, k, nvtxs, ncon, nparts, nbnd, mincut, me, other;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * pwgts, * where, * bndind, * bndptr;

            nparts = ctrl->nparts;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            where = graph->where.data();
            pwgts = iset(nparts * ncon, 0, graph->pwgts.data());
            bndind = graph->bndind.data();
            bndptr = iset(nvtxs, -1, graph->bndptr.data());

            nbnd = mincut = 0;

            /* Compute pwgts */
            if (ncon == 1) {
                for (i = 0; i < nvtxs; i++) {
                    ASSERT(where[i] >= 0 && where[i] < nparts);
                    pwgts[where[i]] += vwgt[i];
                }
            }
            else {
                for (i = 0; i < nvtxs; i++) {
                    me = where[i];
                    for (j = 0; j < ncon; j++)
                        pwgts[me * ncon + j] += vwgt[i * ncon + j];
                }
            }

            /* Compute the required info for refinement */
            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
            {
                ckrinfo_t* myrinfo;
                cnbr_t* mynbrs;

                memset(graph->ckrinfo.data(), 0, sizeof(ckrinfo_t) * nvtxs);
                cnbrpoolReset(ctrl);

                for (i = 0; i < nvtxs; i++) {
                    me = where[i];
                    myrinfo = graph->ckrinfo.data() + i;

                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        if (me == where[adjncy[j]])
                            myrinfo->id += adjwgt[j];
                        else
                            myrinfo->ed += adjwgt[j];
                    }

                    /* Time to compute the particular external degrees */
                    if (myrinfo->ed > 0) {
                        mincut += myrinfo->ed;

                        myrinfo->inbr = cnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                        mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr;

                        for (j = xadj[i]; j < xadj[i + 1]; j++) {
                            other = where[adjncy[j]];
                            if (me != other) {
                                for (k = 0; k < myrinfo->nnbrs; k++) {
                                    if (mynbrs[k].pid == other) {
                                        mynbrs[k].ed += adjwgt[j];
                                        break;
                                    }
                                }
                                if (k == myrinfo->nnbrs) {
                                    mynbrs[k].pid = other;
                                    mynbrs[k].ed = adjwgt[j];
                                    myrinfo->nnbrs++;
                                }
                            }
                        }

                        ASSERT(myrinfo->nnbrs <= xadj[i + 1] - xadj[i]);

                        /* Only ed-id>=0 nodes are considered to be in the boundary */
                        if (myrinfo->ed - myrinfo->id >= 0)
                            BNDInsert(nbnd, bndind, bndptr, i);
                    }
                    else {
                        myrinfo->inbr = -1;
                    }
                }

                graph->mincut = mincut / 2;
                graph->nbnd = nbnd;

            }
            ASSERT(CheckBnd2(graph));
            break;

            case METIS_OBJTYPE_VOL:
            {
                vkrinfo_t* myrinfo;
                vnbr_t* mynbrs;

                memset(graph->vkrinfo.data(), 0, sizeof(vkrinfo_t) * nvtxs);
                vnbrpoolReset(ctrl);

                /* Compute now the id/ed degrees */
                for (i = 0; i < nvtxs; i++) {
                    me = where[i];
                    myrinfo = graph->vkrinfo.data() + i;

                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        if (me == where[adjncy[j]])
                            myrinfo->nid++;
                        else
                            myrinfo->ned++;
                    }

                    /* Time to compute the particular external degrees */
                    if (myrinfo->ned > 0) {
                        mincut += myrinfo->ned;

                        myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                        mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                        for (j = xadj[i]; j < xadj[i + 1]; j++) {
                            other = where[adjncy[j]];
                            if (me != other) {
                                for (k = 0; k < myrinfo->nnbrs; k++) {
                                    if (mynbrs[k].pid == other) {
                                        mynbrs[k].ned++;
                                        break;
                                    }
                                }
                                if (k == myrinfo->nnbrs) {
                                    mynbrs[k].gv = 0;
                                    mynbrs[k].pid = other;
                                    mynbrs[k].ned = 1;
                                    myrinfo->nnbrs++;
                                }
                            }
                        }
                        ASSERT(myrinfo->nnbrs <= xadj[i + 1] - xadj[i]);
                    }
                    else {
                        myrinfo->inbr = -1;
                    }
                }
                graph->mincut = mincut / 2;

                ComputeKWayVolGains(ctrl, graph);
            }
            ASSERT(graph->minvol == ComputeVolume(graph, graph->where.data()));
            break;
            default:
                gk_errexit("Unknown objtype");
            }
        }

        void ProjectKWayPartition(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, j, k, nvtxs, nbnd, nparts, me, other, istart, iend, tid, ted;
            idx_t* xadj, * adjncy, * adjwgt;
            idx_t* cmap, * where, * bndptr, * bndind, * cwhere;
            graph_t* cgraph;
            int dropedges;

            WCOREPUSH;

            dropedges = ctrl->dropedges;

            nparts = ctrl->nparts;

            cgraph = graph->coarser;
            cwhere = cgraph->where.data();

            if (ctrl->objtype == METIS_OBJTYPE_CUT) {
                ASSERT(CheckBnd2(cgraph));
            }
            else {
                ASSERT(cgraph->minvol == ComputeVolume(cgraph, cgraph->where.data()));
            }

            /* free the coarse graph's structure (reduce maxmem) */
            // FreeSData(cgraph);

            nvtxs = graph->nvtxs;
            cmap = graph->cmap.data();
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            AllocateKWayPartitionMemory(ctrl, graph);

            where = graph->where.data();
            bndind = graph->bndind.data();
            bndptr = iset(nvtxs, -1, graph->bndptr.data());

            std::vector<idx_t> htable(nparts, -1);

            /* Compute the required info for refinement */
            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
            {
                ckrinfo_t* myrinfo;
                cnbr_t* mynbrs;

                /* go through and project partition and compute id/ed for the nodes */
                for (i = 0; i < nvtxs; i++) {
                    k = cmap[i];
                    where[i] = cwhere[k];
                    cmap[i] = (dropedges ? 1 : cgraph->ckrinfo[k].ed);  /* For optimization */
                }

                memset(graph->ckrinfo.data(), 0, sizeof(ckrinfo_t) * nvtxs);
                cnbrpoolReset(ctrl);

                for (nbnd = 0, i = 0; i < nvtxs; i++) {
                    istart = xadj[i];
                    iend = xadj[i + 1];

                    myrinfo = graph->ckrinfo.data() + i;

                    if (cmap[i] == 0) { /* Interior node. Note that cmap[i] = crinfo[cmap[i]].ed */
                        for (tid = 0, j = istart; j < iend; j++)
                            tid += adjwgt[j];

                        myrinfo->id = tid;
                        myrinfo->inbr = -1;
                    }
                    else { /* Potentially an interface node */
                        myrinfo->inbr = cnbrpoolGetNext(ctrl, iend - istart);
                        mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr;

                        me = where[i];
                        for (tid = 0, ted = 0, j = istart; j < iend; j++) {
                            other = where[adjncy[j]];
                            if (me == other) {
                                tid += adjwgt[j];
                            }
                            else {
                                ted += adjwgt[j];
                                if ((k = htable[other]) == -1) {
                                    htable[other] = myrinfo->nnbrs;
                                    mynbrs[myrinfo->nnbrs].pid = other;
                                    mynbrs[myrinfo->nnbrs++].ed = adjwgt[j];
                                }
                                else {
                                    mynbrs[k].ed += adjwgt[j];
                                }
                            }
                        }
                        myrinfo->id = tid;
                        myrinfo->ed = ted;

                        /* Remove space for edegrees if it was interior */
                        if (ted == 0) {
                            ctrl->nbrpoolcpos -= gk_min(nparts, iend - istart);
                            myrinfo->inbr = -1;
                        }
                        else {
                            if (ted - tid >= 0)
                                BNDInsert(nbnd, bndind, bndptr, i);

                            for (j = 0; j < myrinfo->nnbrs; j++)
                                htable[mynbrs[j].pid] = -1;
                        }
                    }
                }

                graph->nbnd = nbnd;
            }
            ASSERT(CheckBnd2(graph));
            break;

            case METIS_OBJTYPE_VOL:
            {
                vkrinfo_t* myrinfo;
                vnbr_t* mynbrs;

                /* go through and project partition and compute id/ed for the nodes */
                for (i = 0; i < nvtxs; i++) {
                    k = cmap[i];
                    where[i] = cwhere[k];
                    cmap[i] = (dropedges ? 1 : cgraph->vkrinfo[k].ned);  /* For optimization */
                }

                memset(graph->vkrinfo.data(), 0, sizeof(vkrinfo_t) * nvtxs);
                vnbrpoolReset(ctrl);

                for (i = 0; i < nvtxs; i++) {
                    istart = xadj[i];
                    iend = xadj[i + 1];
                    myrinfo = graph->vkrinfo.data() + i;

                    if (cmap[i] == 0) { /* Note that cmap[i] = crinfo[cmap[i]].ed */
                        myrinfo->nid = iend - istart;
                        myrinfo->inbr = -1;
                    }
                    else { /* Potentially an interface node */
                        myrinfo->inbr = vnbrpoolGetNext(ctrl, iend - istart);
                        mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                        me = where[i];
                        for (tid = 0, ted = 0, j = istart; j < iend; j++) {
                            other = where[adjncy[j]];
                            if (me == other) {
                                tid++;
                            }
                            else {
                                ted++;
                                if ((k = htable[other]) == -1) {
                                    htable[other] = myrinfo->nnbrs;
                                    mynbrs[myrinfo->nnbrs].gv = 0;
                                    mynbrs[myrinfo->nnbrs].pid = other;
                                    mynbrs[myrinfo->nnbrs++].ned = 1;
                                }
                                else {
                                    mynbrs[k].ned++;
                                }
                            }
                        }
                        myrinfo->nid = tid;
                        myrinfo->ned = ted;

                        /* Remove space for edegrees if it was interior */
                        if (ted == 0) {
                            ctrl->nbrpoolcpos -= gk_min(nparts, iend - istart);
                            myrinfo->inbr = -1;
                        }
                        else {
                            for (j = 0; j < myrinfo->nnbrs; j++)
                                htable[mynbrs[j].pid] = -1;
                        }
                    }
                }

                ComputeKWayVolGains(ctrl, graph);

                ASSERT(graph->minvol == ComputeVolume(graph, graph->where.data()));
            }
            break;

            default:
                gk_errexit("Unknown objtype of %d\n");
            }

            graph->mincut = (dropedges ? ComputeCut(graph, where) : cgraph->mincut);
            icopy(nparts * graph->ncon, cgraph->pwgts.data(), graph->pwgts.data());

            delete graph->coarser;
            graph->coarser = NULL;

            WCOREPOP;
        }


        void ComputeKWayBoundary(ctrl_t* ctrl, graph_t* graph, idx_t bndtype)
        {
            idx_t i, nvtxs, nbnd;
            idx_t* bndind, * bndptr;

            nvtxs = graph->nvtxs;
            bndind = graph->bndind.data();
            bndptr = iset(nvtxs, -1, graph->bndptr.data());

            nbnd = 0;

            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
                /* Compute the boundary */
                if (bndtype == BNDTYPE_REFINE) {
                    for (i = 0; i < nvtxs; i++) {
                        if (graph->ckrinfo[i].ed > 0 && graph->ckrinfo[i].ed - graph->ckrinfo[i].id >= 0)
                            BNDInsert(nbnd, bndind, bndptr, i);
                    }
                }
                else { /* BNDTYPE_BALANCE */
                    for (i = 0; i < nvtxs; i++) {
                        if (graph->ckrinfo[i].ed > 0)
                            BNDInsert(nbnd, bndind, bndptr, i);
                    }
                }
                break;

            case METIS_OBJTYPE_VOL:
                /* Compute the boundary */
                if (bndtype == BNDTYPE_REFINE) {
                    for (i = 0; i < nvtxs; i++) {
                        if (graph->vkrinfo[i].gv >= 0)
                            BNDInsert(nbnd, bndind, bndptr, i);
                    }
                }
                else { /* BNDTYPE_BALANCE */
                    for (i = 0; i < nvtxs; i++) {
                        if (graph->vkrinfo[i].ned > 0)
                            BNDInsert(nbnd, bndind, bndptr, i);
                    }
                }
                break;

            default:
                gk_errexit("Unknown objtype");
            }

            graph->nbnd = nbnd;
        }

        void ComputeKWayVolGains(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, ii, j, k, nvtxs, nparts, me, other;
            idx_t* xadj, * vsize, * adjncy, * where,
                * bndind, * bndptr;
            vkrinfo_t* myrinfo, * orinfo;
            vnbr_t* mynbrs, * onbrs;

            WCOREPUSH;

            nparts = ctrl->nparts;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vsize = graph->vsize.data();
            adjncy = graph->adjncy.data();

            where = graph->where.data();
            bndind = graph->bndind.data();
            bndptr = iset(nvtxs, -1, graph->bndptr.data());

            std::vector<idx_t> ophtable(nparts, -1);

            /* Compute the volume gains */
            graph->minvol = graph->nbnd = 0;
            for (i = 0; i < nvtxs; i++) {
                myrinfo = graph->vkrinfo.data() + i;
                myrinfo->gv = IDX_MIN;

                if (myrinfo->nnbrs > 0) {
                    me = where[i];
                    mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                    graph->minvol += myrinfo->nnbrs * vsize[i];

                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        ii = adjncy[j];
                        other = where[ii];
                        orinfo = graph->vkrinfo.data() + ii;
                        onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                        for (k = 0; k < orinfo->nnbrs; k++)
                            ophtable[onbrs[k].pid] = k;
                        ophtable[other] = 1;  /* this is to simplify coding */

                        if (me == other) {
                            /* Find which domains 'i' is connected to but 'ii' is not
                               and update their gain */
                            for (k = 0; k < myrinfo->nnbrs; k++) {
                                if (ophtable[mynbrs[k].pid] == -1)
                                    mynbrs[k].gv -= vsize[ii];
                            }
                        }
                        else {
                            ASSERT(ophtable[me] != -1);

                            if (onbrs[ophtable[me]].ned == 1) {
                                /* I'm the only connection of 'ii' in 'me' */
                                /* Increase the gains for all the common domains between 'i' and 'ii' */
                                for (k = 0; k < myrinfo->nnbrs; k++) {
                                    if (ophtable[mynbrs[k].pid] != -1)
                                        mynbrs[k].gv += vsize[ii];
                                }
                            }
                            else {
                                /* Find which domains 'i' is connected to and 'ii' is not
                                   and update their gain */
                                for (k = 0; k < myrinfo->nnbrs; k++) {
                                    if (ophtable[mynbrs[k].pid] == -1)
                                        mynbrs[k].gv -= vsize[ii];
                                }
                            }
                        }

                        /* Reset the marker vector */
                        for (k = 0; k < orinfo->nnbrs; k++)
                            ophtable[onbrs[k].pid] = -1;
                        ophtable[other] = -1;
                    }

                    /* Compute the max vgain */
                    for (k = 0; k < myrinfo->nnbrs; k++) {
                        if (mynbrs[k].gv > myrinfo->gv)
                            myrinfo->gv = mynbrs[k].gv;
                    }

                    /* Add the extra gain due to id == 0 */
                    if (myrinfo->ned > 0 && myrinfo->nid == 0)
                        myrinfo->gv += vsize[i];
                }

                if (myrinfo->gv >= 0)
                    BNDInsert(graph->nbnd, bndind, bndptr, i);
            }

            WCOREPOP;
        }

        void Greedy_KWayOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter,
            real_t ffactor, idx_t omode)
        {
            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
                if (graph->ncon == 1)
                    Greedy_KWayCutOptimize(ctrl, graph, niter, ffactor, omode);
                else
                    Greedy_McKWayCutOptimize(ctrl, graph, niter, ffactor, omode);
                break;

            case METIS_OBJTYPE_VOL:
                if (graph->ncon == 1)
                    Greedy_KWayVolOptimize(ctrl, graph, niter, ffactor, omode);
                else
                    Greedy_McKWayVolOptimize(ctrl, graph, niter, ffactor, omode);
                break;

            default:
                gk_errexit("Unknown objtype");
            }
        }

        void Greedy_KWayCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter,
            real_t ffactor, idx_t omode)
        {
            /* Common variables to all types of kway-refinement/balancing routines */
            idx_t i, ii, iii, j, k, pass, nvtxs, nparts, gain;
            idx_t from, me, to, oldcut, vwgt;
            idx_t* xadj, * adjncy, * adjwgt;
            idx_t* where, * pwgts, * bndptr, * bndind;
            idx_t nmoved, nupd;
            idx_t maxndoms = 0, * nads = NULL, * doms = NULL;
            std::vector<idx_t>* adids = NULL;
            idx_t* bfslvl = NULL, * bfsind = NULL, * bfsmrk = NULL;
            idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
            real_t* tpwgts, ubfactor;

            /* Edgecut-specific/different variables */
            idx_t nbnd, oldnnbrs;
            rpq_t* queue;
            real_t rgain;
            ckrinfo_t* myrinfo;
            cnbr_t* mynbrs;

            ffactor = 0.0;
            WCOREPUSH;

            /* Link the graph fields */
            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            bndind = graph->bndind.data();
            bndptr = graph->bndptr.data();

            where = graph->where.data();
            pwgts = graph->pwgts.data();

            nparts = ctrl->nparts;
            tpwgts = ctrl->tpwgts.data();

            /* Setup the weight intervals of the various subdomains */
            std::vector<idx_t> minpwgts(nparts);
            std::vector<idx_t> maxpwgts(nparts);

            if (omode == OMODE_BALANCE)
                ubfactor = ctrl->ubfactors[0];
            else
                ubfactor = gk_max(ctrl->ubfactors[0], ComputeLoadImbalance(graph, nparts, ctrl->pijbm.data()));

            for (i = 0; i < nparts; i++) {
                maxpwgts[i] = (idx_t)(tpwgts[i] * graph->tvwgt[0] * ubfactor);
                minpwgts[i] = (idx_t)(tpwgts[i] * graph->tvwgt[0] * (1.0f / ubfactor));
            }

            std::vector<idx_t> perm(nvtxs);

            /* This stores the valid target subdomains. It is used when ctrl->minconn to
               control the subdomains to which moves are allowed to be made.
               When ctrl->minconn is false, the default values of 2 allow all moves to
               go through and it does not interfere with the zero-gain move selection. */
            std::vector<idx_t> safetos(nparts, 2);

            if (ctrl->minconn) {
                ComputeSubDomainGraph(ctrl, graph);

                nads = ctrl->nads.data();
                adids = ctrl->adids.data();
                doms = iset(nparts, 0, ctrl->pvec1.data());
            }


            /* Setup updptr, updind like boundary info to keep track of the vertices whose
               vstatus's need to be reset at the end of the inner iteration */
            std::vector<idx_t> vstatus(nvtxs, VPQSTATUS_NOTPRESENT);
            std::vector<idx_t> updptr(nvtxs, -1);
            std::vector<idx_t> updind(nvtxs);

            std::vector<idx_t> v_bfslvl, v_bfsind, v_bfsmrk;
            if (ctrl->contig) {
                /* The arrays that will be used for limited check of articulation points */
                v_bfslvl.resize(nvtxs);
                v_bfsind.resize(nvtxs);
                v_bfsmrk.resize(nvtxs);

                bfslvl = iset(nvtxs, 0, v_bfslvl.data());
                bfsind = v_bfsind.data();
                bfsmrk = iset(nvtxs, 0, v_bfsmrk.data());
            }

            queue = rpqCreate(nvtxs);

            /*=====================================================================
            * The top-level refinement loop
            *======================================================================*/
            for (pass = 0; pass < niter; pass++) {
                ASSERT(ComputeCut(graph, where) == graph->mincut);
                if (omode == OMODE_REFINE)
                    ASSERT(CheckBnd2(graph));

                if (omode == OMODE_BALANCE) {
                    /* Check to see if things are out of balance, given the tolerance */
                    for (i = 0; i < nparts; i++) {
                        if (pwgts[i] > maxpwgts[i] || pwgts[i] < minpwgts[i])
                            break;
                    }
                    if (i == nparts) /* Things are balanced. Return right away */
                        break;
                }

                oldcut = graph->mincut;
                nbnd = graph->nbnd;
                nupd = 0;

                if (ctrl->minconn)
                    maxndoms = imax(nparts, nads, 1);

                /* Insert the boundary vertices in the priority queue */
                irandArrayPermute(nbnd, perm.data(), nbnd / 4, 1);
                for (ii = 0; ii < nbnd; ii++) {
                    i = bndind[perm[ii]];
                    rgain = (graph->ckrinfo[i].nnbrs > 0 ?
                        1.0f * graph->ckrinfo[i].ed / sqrtf((real_t)graph->ckrinfo[i].nnbrs) : 0.0f)
                        - graph->ckrinfo[i].id;
                    rpqInsert(queue, i, rgain);
                    vstatus[i] = VPQSTATUS_PRESENT;
                    ListInsert(nupd, updind, updptr, i);
                }

                /* Start extracting vertices from the queue and try to move them */
                for (nmoved = 0, iii = 0;; iii++) {
                    if ((i = rpqGetTop(queue)) == -1)
                        break;
                    vstatus[i] = VPQSTATUS_EXTRACTED;

                    myrinfo = graph->ckrinfo.data() + i;
                    mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr;

                    from = where[i];
                    vwgt = graph->vwgt[i];

                    if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                        continue;

                    if (ctrl->minconn)
                        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

                    /* Find the most promising subdomain to move to */
                    if (omode == OMODE_REFINE) {
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            if (((mynbrs[k].ed > myrinfo->id) &&
                                ((pwgts[from] - vwgt >= minpwgts[from]) ||
                                    (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))) &&
                                ((pwgts[to] + vwgt <= maxpwgts[to]) ||
                                    (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt)))
                                ) ||
                                ((mynbrs[k].ed == myrinfo->id) &&
                                    (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt)))
                                )
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            if (((mynbrs[j].ed > mynbrs[k].ed) &&
                                ((pwgts[from] - vwgt >= minpwgts[from]) ||
                                    (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))) &&
                                ((pwgts[to] + vwgt <= maxpwgts[to]) ||
                                    (tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt)))
                                ) ||
                                ((mynbrs[j].ed == mynbrs[k].ed) &&
                                    (tpwgts[mynbrs[k].pid] * pwgts[to] < tpwgts[to] * pwgts[mynbrs[k].pid]))
                                )
                                k = j;
                        }

                        to = mynbrs[k].pid;

                        gain = mynbrs[k].ed - myrinfo->id;
                        (void)gain;
                        /*
                        if (!(gain > 0
                              || (gain == 0
                                  && (pwgts[from] >= maxpwgts[from]
                                      || tpwgts[to]*pwgts[from] > tpwgts[from]*(pwgts[to]+vwgt)
                                      || (iii%2 == 0 && safetos[to] == 2)
                                     )
                                 )
                             )
                           )
                          continue;
                        */
                    }
                    else {  /* OMODE_BALANCE */
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            /* the correctness of the following test follows from the correctness
                               of the similar test in the subsequent loop */
                            if (from >= nparts || tpwgts[from] * pwgts[to] < tpwgts[to] * (pwgts[from] - vwgt))
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            if (tpwgts[mynbrs[k].pid] * pwgts[to] < tpwgts[to] * pwgts[mynbrs[k].pid])
                                k = j;
                        }

                        to = mynbrs[k].pid;

                        //if (pwgts[from] < maxpwgts[from] && pwgts[to] > minpwgts[to] && 
                        //    mynbrs[k].ed-myrinfo->id < 0) 
                        //  continue;
                    }


                    /*=====================================================================
                    * If we got here, we can now move the vertex from 'from' to 'to'
                    *======================================================================*/
                    graph->mincut -= mynbrs[k].ed - myrinfo->id;
                    nmoved++;

                    /* Update the subdomain connectivity information */
                    if (ctrl->minconn) {
                        /* take care of i's move itself */
                        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id - mynbrs[k].ed, &maxndoms);

                        /* take care of the adjacent vertices */
                        for (j = xadj[i]; j < xadj[i + 1]; j++) {
                            me = where[adjncy[j]];
                            if (me != from && me != to) {
                                UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], &maxndoms);
                                UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], &maxndoms);
                            }
                        }
                    }

                    /* Update ID/ED and BND related information for the moved vertex */
                    INC_DEC(pwgts[to], pwgts[from], vwgt);
                    UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, nbnd,
                        bndptr, bndind, bndtype);

                    /* Update the degrees of adjacent vertices */
                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        ii = adjncy[j];
                        me = where[ii];
                        myrinfo = graph->ckrinfo.data() + ii;

                        oldnnbrs = myrinfo->nnbrs;

                        UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii + 1] - xadj[ii], me,
                            from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

                        UpdateQueueInfo(queue, vstatus, ii, me, from, to, myrinfo, oldnnbrs,
                            nupd, updptr, updind, bndtype);

                        ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
                    }

                }

                graph->nbnd = nbnd;

                /* Reset the vstatus and associated data structures */
                for (i = 0; i < nupd; i++) {
                    ASSERT(updptr[updind[i]] != -1);
                    ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
                    vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
                    updptr[updind[i]] = -1;
                }

                if (nmoved == 0 || (omode == OMODE_REFINE && graph->mincut == oldcut))
                    break;
            }

            rpqDestroy(queue);

            WCOREPOP;
        }

        void Greedy_McKWayCutOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter,
            real_t ffactor, idx_t omode)
        {
            /* Common variables to all types of kway-refinement/balancing routines */
            idx_t i, ii, iii, j, k, pass, nvtxs, ncon, nparts, gain;
            idx_t from, me, to, cto, oldcut;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt;
            idx_t* where, * pwgts, * bndptr, * bndind;
            idx_t nmoved, nupd;
            idx_t maxndoms = 0, * nads = NULL, * doms = NULL;
            std::vector<idx_t>* adids = NULL;
            idx_t* bfslvl = NULL, * bfsind = NULL, * bfsmrk = NULL;
            idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
            real_t* pijbm;

            /* Edgecut-specific/different variables */
            idx_t nbnd, oldnnbrs;
            rpq_t* queue;
            real_t rgain;
            ckrinfo_t* myrinfo;
            cnbr_t* mynbrs;

            WCOREPUSH;

            /* Link the graph fields */
            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            bndind = graph->bndind.data();
            bndptr = graph->bndptr.data();

            where = graph->where.data();
            pwgts = graph->pwgts.data();

            nparts = ctrl->nparts;
            pijbm = ctrl->pijbm.data();


            /* Determine the ubfactors. The method used is different based on omode.
               When OMODE_BALANCE, the ubfactors are those supplied by the user.
               When OMODE_REFINE, the ubfactors are the max of the current partition
               and the user-specified ones. */
            std::vector<real_t> ubfactors(ncon);

            ComputeLoadImbalanceVec(graph, nparts, pijbm, ubfactors.data());
            real_t origbal = rvecmaxdiff(ncon, ubfactors.data(), ctrl->ubfactors.data());
            (void)origbal;

            if (omode == OMODE_BALANCE) {
                rcopy(ncon, ctrl->ubfactors.data(), ubfactors.data());
            }
            else {
                for (i = 0; i < ncon; i++)
                    ubfactors[i] = (ubfactors[i] > ctrl->ubfactors[i] ? ubfactors[i] : ctrl->ubfactors[i]);
            }

            /* Setup the weight intervals of the various subdomains */
            std::vector<idx_t> minpwgts(nparts * ncon);
            std::vector<idx_t> maxpwgts(nparts * ncon);

            for (i = 0; i < nparts; i++) {
                for (j = 0; j < ncon; j++) {
                    maxpwgts[i * ncon + j] = (idx_t)(ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * ubfactors[j]);
                    /*minpwgts[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*(.9/ubfactors[j]);*/
                    minpwgts[i * ncon + j] = (idx_t)(ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * 0.2f);
                }
            }

            std::vector<idx_t> perm(nvtxs);

            /* This stores the valid target subdomains. It is used when ctrl->minconn to
               control the subdomains to which moves are allowed to be made.
               When ctrl->minconn is false, the default values of 2 allow all moves to
               go through and it does not interfere with the zero-gain move selection. */
            std::vector<idx_t> safetos(nparts, 2);

            if (ctrl->minconn) {
                ComputeSubDomainGraph(ctrl, graph);

                nads = ctrl->nads.data();
                adids = ctrl->adids.data();
                // adwgts = ctrl->adwgts.data();
                doms = iset(nparts, 0, ctrl->pvec1.data());
            }


            /* Setup updptr, updind like boundary info to keep track of the vertices whose
               vstatus's need to be reset at the end of the inner iteration */

            std::vector<idx_t> vstatus(nvtxs, VPQSTATUS_NOTPRESENT);
            std::vector<idx_t> updptr(nvtxs, -1);
            std::vector<idx_t> updind(nvtxs);

            std::vector<idx_t> v_bfslvl, v_bfsind, v_bfsmrk;
            if (ctrl->contig) {
                /* The arrays that will be used for limited check of articulation points */
                v_bfslvl.resize(nvtxs);
                v_bfsind.resize(nvtxs);
                v_bfsmrk.resize(nvtxs);

                bfslvl = iset(nvtxs, 0, v_bfslvl.data());
                bfsind = v_bfsind.data();
                bfsmrk = iset(nvtxs, 0, v_bfsmrk.data());
            }

            queue = rpqCreate(nvtxs);


            /*=====================================================================
            * The top-level refinement loop
            *======================================================================*/
            for (pass = 0; pass < niter; pass++) {
                ASSERT(ComputeCut(graph, where) == graph->mincut);
                if (omode == OMODE_REFINE)
                    ASSERT(CheckBnd2(graph));

                /* In balancing mode, exit as soon as balance is reached */
                if (omode == OMODE_BALANCE && IsBalanced(ctrl, graph, 0))
                    break;

                oldcut = graph->mincut;
                nbnd = graph->nbnd;
                nupd = 0;

                if (ctrl->minconn)
                    maxndoms = imax(nparts, nads, 1);

                /* Insert the boundary vertices in the priority queue */
                irandArrayPermute(nbnd, perm.data(), nbnd / 4, 1);
                for (ii = 0; ii < nbnd; ii++) {
                    i = bndind[perm[ii]];
                    rgain = (graph->ckrinfo[i].nnbrs > 0 ?
                        1.0f * graph->ckrinfo[i].ed / sqrtf((real_t)graph->ckrinfo[i].nnbrs) : 0.0f)
                        - graph->ckrinfo[i].id;
                    rpqInsert(queue, i, rgain);
                    vstatus[i] = VPQSTATUS_PRESENT;
                    ListInsert(nupd, updind, updptr, i);
                }

                /* Start extracting vertices from the queue and try to move them */
                for (nmoved = 0, iii = 0;; iii++) {
                    if ((i = rpqGetTop(queue)) == -1)
                        break;
                    vstatus[i] = VPQSTATUS_EXTRACTED;

                    myrinfo = graph->ckrinfo.data() + i;
                    mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr;

                    from = where[i];

                    /* Prevent moves that make 'from' domain underbalanced */
                    if (omode == OMODE_REFINE) {
                        if (myrinfo->id > 0 &&
                            !ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts.data() + from * ncon))
                            continue;
                    }
                    else { /* OMODE_BALANCE */
                        if (!ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts.data() + from * ncon))
                            continue;
                    }

                    if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                        continue;

                    if (ctrl->minconn)
                        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

                    /* Find the most promising subdomain to move to */
                    if (omode == OMODE_REFINE) {
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            gain = mynbrs[k].ed - myrinfo->id;
                            if (gain >= 0 && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon))
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        cto = to;
                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            if ((mynbrs[j].ed > mynbrs[k].ed &&
                                ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon))
                                ||
                                (mynbrs[j].ed == mynbrs[k].ed &&
                                    BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                        1, pwgts + cto * ncon, pijbm + cto * ncon,
                                        1, pwgts + to * ncon, pijbm + to * ncon))) {
                                k = j;
                                cto = to;
                            }
                        }
                        to = cto;

                        gain = mynbrs[k].ed - myrinfo->id;
                        if (!(gain > 0
                            || (gain == 0
                                && (BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                    -1, pwgts + from * ncon, pijbm + from * ncon,
                                    +1, pwgts + to * ncon, pijbm + to * ncon)
                                    || (iii % 2 == 0 && safetos[to] == 2)
                                    )
                                )
                            )
                            )
                            continue;
                    }
                    else {  /* OMODE_BALANCE */
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            if (ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon) ||
                                BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                    -1, pwgts + from * ncon, pijbm + from * ncon,
                                    +1, pwgts + to * ncon, pijbm + to * ncon))
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        cto = to;
                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            if (BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                1, pwgts + cto * ncon, pijbm + cto * ncon,
                                1, pwgts + to * ncon, pijbm + to * ncon)) {
                                k = j;
                                cto = to;
                            }
                        }
                        to = cto;

                        if (mynbrs[k].ed - myrinfo->id < 0 &&
                            !BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                -1, pwgts + from * ncon, pijbm + from * ncon,
                                +1, pwgts + to * ncon, pijbm + to * ncon))
                            continue;
                    }



                    /*=====================================================================
                    * If we got here, we can now move the vertex from 'from' to 'to'
                    *======================================================================*/
                    graph->mincut -= mynbrs[k].ed - myrinfo->id;
                    nmoved++;

                    /* Update the subdomain connectivity information */
                    if (ctrl->minconn) {
                        /* take care of i's move itself */
                        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id - mynbrs[k].ed, &maxndoms);

                        /* take care of the adjacent vertices */
                        for (j = xadj[i]; j < xadj[i + 1]; j++) {
                            me = where[adjncy[j]];
                            if (me != from && me != to) {
                                UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], &maxndoms);
                                UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], &maxndoms);
                            }
                        }
                    }

                    /* Update ID/ED and BND related information for the moved vertex */
                    iaxpy(ncon, 1, vwgt + i * ncon, 1, pwgts + to * ncon, 1);
                    iaxpy(ncon, -1, vwgt + i * ncon, 1, pwgts + from * ncon, 1);
                    UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where,
                        nbnd, bndptr, bndind, bndtype);

                    /* Update the degrees of adjacent vertices */
                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        ii = adjncy[j];
                        me = where[ii];
                        myrinfo = graph->ckrinfo.data() + ii;

                        oldnnbrs = myrinfo->nnbrs;

                        UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii + 1] - xadj[ii], me,
                            from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

                        UpdateQueueInfo(queue, vstatus, ii, me, from, to, myrinfo, oldnnbrs,
                            nupd, updptr, updind, bndtype);

                        ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
                    }
                }

                graph->nbnd = nbnd;

                /* Reset the vstatus and associated data structures */
                for (i = 0; i < nupd; i++) {
                    ASSERT(updptr[updind[i]] != -1);
                    ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
                    vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
                    updptr[updind[i]] = -1;
                }

                if (nmoved == 0 || (omode == OMODE_REFINE && graph->mincut == oldcut))
                    break;
            }

            rpqDestroy(queue);

            WCOREPOP;
        }


        void Greedy_KWayVolOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter,
            real_t ffactor, idx_t omode)
        {
            /* Common variables to all types of kway-refinement/balancing routines */
            idx_t i, ii, iii, j, k, pass, nvtxs, nparts, gain;
            idx_t from, me, to, oldcut, vwgt;
            idx_t* xadj, * adjncy;
            idx_t* where, * pwgts, * bndind;
            idx_t nmoved, nupd;
            idx_t maxndoms = 0, * nads = NULL, * doms = NULL;
            std::vector<idx_t>* adids = NULL;
            idx_t* bfslvl = NULL, * bfsind = NULL, * bfsmrk = NULL;
            idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
            real_t* tpwgts;

            /* Volume-specific/different variables */
            ipq_t* queue;
            idx_t oldvol, xgain;
            vkrinfo_t* myrinfo;
            vnbr_t* mynbrs;

            WCOREPUSH;

            /* Link the graph fields */
            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            bndind = graph->bndind.data();
            where = graph->where.data();
            pwgts = graph->pwgts.data();

            nparts = ctrl->nparts;
            tpwgts = ctrl->tpwgts.data();

            /* Setup the weight intervals of the various subdomains */
            std::vector<idx_t> minpwgts(nparts);
            std::vector<idx_t> maxpwgts(nparts);

            for (i = 0; i < nparts; i++) {
                maxpwgts[i] = (idx_t)(ctrl->tpwgts[i] * graph->tvwgt[0] * ctrl->ubfactors[0]);
                minpwgts[i] = (idx_t)(ctrl->tpwgts[i] * graph->tvwgt[0] * (1.0f / ctrl->ubfactors[0]));
            }

            std::vector<idx_t> perm(nvtxs);

            /* This stores the valid target subdomains. It is used when ctrl->minconn to
               control the subdomains to which moves are allowed to be made.
               When ctrl->minconn is false, the default values of 2 allow all moves to
               go through and it does not interfere with the zero-gain move selection. */
            std::vector<idx_t> safetos(nparts, 2);

            if (ctrl->minconn) {
                ComputeSubDomainGraph(ctrl, graph);

                nads = ctrl->nads.data();
                adids = ctrl->adids.data();
                doms = iset(nparts, 0, ctrl->pvec1.data());
            }


            /* Setup updptr, updind like boundary info to keep track of the vertices whose
               vstatus's need to be reset at the end of the inner iteration */
            std::vector<idx_t> vstatus(nvtxs, VPQSTATUS_NOTPRESENT);
            std::vector<idx_t> updptr(nvtxs, -1);
            std::vector<idx_t> updind(nvtxs);

            std::vector<idx_t> v_bfslvl, v_bfsind, v_bfsmrk;
            if (ctrl->contig) {
                /* The arrays that will be used for limited check of articulation points */
                v_bfslvl.resize(nvtxs);
                v_bfsind.resize(nvtxs);
                v_bfsmrk.resize(nvtxs);

                bfslvl = iset(nvtxs, 0, v_bfslvl.data());
                bfsind = v_bfsind.data();
                bfsmrk = iset(nvtxs, 0, v_bfsmrk.data());
            }

            /* Vol-refinement specific working arrays */
            std::vector<idx_t> modind(nvtxs);
            std::vector<idx_t> vmarker(nvtxs, 0);
            std::vector<idx_t> pmarker(nparts, -1);

            queue = ipqCreate(nvtxs);

            /*=====================================================================
            * The top-level refinement loop
            *======================================================================*/
            for (pass = 0; pass < niter; pass++) {
                ASSERT(ComputeVolume(graph, where) == graph->minvol);

                if (omode == OMODE_BALANCE) {
                    /* Check to see if things are out of balance, given the tolerance */
                    for (i = 0; i < nparts; i++) {
                        if (pwgts[i] > maxpwgts[i])
                            break;
                    }
                    if (i == nparts) /* Things are balanced. Return right away */
                        break;
                }

                oldcut = graph->mincut;
                oldvol = graph->minvol;
                nupd = 0;

                if (ctrl->minconn)
                    maxndoms = imax(nparts, nads, 1);

                /* Insert the boundary vertices in the priority queue */
                irandArrayPermute(graph->nbnd, perm.data(), graph->nbnd / 4, 1);
                for (ii = 0; ii < graph->nbnd; ii++) {
                    i = bndind[perm[ii]];
                    ipqInsert(queue, i, graph->vkrinfo[i].gv);
                    vstatus[i] = VPQSTATUS_PRESENT;
                    ListInsert(nupd, updind, updptr, i);
                }

                /* Start extracting vertices from the queue and try to move them */
                for (nmoved = 0, iii = 0;; iii++) {
                    if ((i = ipqGetTop(queue)) == -1)
                        break;
                    vstatus[i] = VPQSTATUS_EXTRACTED;

                    myrinfo = graph->vkrinfo.data() + i;
                    mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                    from = where[i];
                    vwgt = graph->vwgt[i];

                    /* Prevent moves that make 'from' domain underbalanced */
                    if (omode == OMODE_REFINE) {
                        if (myrinfo->nid > 0 && pwgts[from] - vwgt < minpwgts[from])
                            continue;
                    }
                    else { /* OMODE_BALANCE */
                        if (pwgts[from] - vwgt < minpwgts[from])
                            continue;
                    }

                    if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                        continue;

                    if (ctrl->minconn)
                        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

                    xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? graph->vsize[i] : 0);

                    /* Find the most promising subdomain to move to */
                    if (omode == OMODE_REFINE) {
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            gain = mynbrs[k].gv + xgain;
                            if (gain >= 0 && pwgts[to] + vwgt <= maxpwgts[to] + ffactor * gain)
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            gain = mynbrs[j].gv + xgain;
                            if ((mynbrs[j].gv > mynbrs[k].gv &&
                                pwgts[to] + vwgt <= maxpwgts[to] + ffactor * gain)
                                ||
                                (mynbrs[j].gv == mynbrs[k].gv &&
                                    mynbrs[j].ned > mynbrs[k].ned &&
                                    pwgts[to] + vwgt <= maxpwgts[to])
                                ||
                                (mynbrs[j].gv == mynbrs[k].gv &&
                                    mynbrs[j].ned == mynbrs[k].ned &&
                                    tpwgts[mynbrs[k].pid] * pwgts[to] < tpwgts[to] * pwgts[mynbrs[k].pid])
                                )
                                k = j;
                        }
                        to = mynbrs[k].pid;

                        ASSERT(xgain + mynbrs[k].gv >= 0);

                        j = 0;
                        if (xgain + mynbrs[k].gv > 0 || mynbrs[k].ned - myrinfo->nid > 0)
                            j = 1;
                        else if (mynbrs[k].ned - myrinfo->nid == 0) {
                            if ((iii % 2 == 0 && safetos[to] == 2) ||
                                pwgts[from] >= maxpwgts[from] ||
                                tpwgts[from] * (pwgts[to] + vwgt) < tpwgts[to] * pwgts[from])
                                j = 1;
                        }
                        if (j == 0)
                            continue;
                    }
                    else { /* OMODE_BALANCE */
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            if (pwgts[to] + vwgt <= maxpwgts[to] ||
                                tpwgts[from] * (pwgts[to] + vwgt) <= tpwgts[to] * pwgts[from])
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            if (tpwgts[mynbrs[k].pid] * pwgts[to] < tpwgts[to] * pwgts[mynbrs[k].pid])
                                k = j;
                        }
                        to = mynbrs[k].pid;

                        if (pwgts[from] < maxpwgts[from] && pwgts[to] > minpwgts[to] &&
                            (xgain + mynbrs[k].gv < 0 ||
                                (xgain + mynbrs[k].gv == 0 && mynbrs[k].ned - myrinfo->nid < 0))
                            )
                            continue;
                    }


                    /*=====================================================================
                    * If we got here, we can now move the vertex from 'from' to 'to'
                    *======================================================================*/
                    INC_DEC(pwgts[to], pwgts[from], vwgt);
                    graph->mincut -= mynbrs[k].ned - myrinfo->nid;
                    graph->minvol -= (xgain + mynbrs[k].gv);
                    where[i] = to;
                    nmoved++;

                    /* Update the subdomain connectivity information */
                    if (ctrl->minconn) {
                        /* take care of i's move itself */
                        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->nid - mynbrs[k].ned, &maxndoms);

                        /* take care of the adjacent vertices */
                        for (j = xadj[i]; j < xadj[i + 1]; j++) {
                            me = where[adjncy[j]];
                            if (me != from && me != to) {
                                UpdateEdgeSubDomainGraph(ctrl, from, me, -1, &maxndoms);
                                UpdateEdgeSubDomainGraph(ctrl, to, me, 1, &maxndoms);
                            }
                        }
                    }

                    /* Update the id/ed/gains/bnd/queue of potentially affected nodes */
                    KWayVolUpdate(ctrl, graph, i, from, to, queue, vstatus.data(), &nupd, updptr.data(),
                        updind.data(), bndtype, vmarker.data(), pmarker.data(), modind.data());

                    /*CheckKWayVolPartitionParams(ctrl, graph); */
                }


                /* Reset the vstatus and associated data structures */
                for (i = 0; i < nupd; i++) {
                    ASSERT(updptr[updind[i]] != -1);
                    ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
                    vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
                    updptr[updind[i]] = -1;
                }

                if (nmoved == 0 ||
                    (omode == OMODE_REFINE && graph->minvol == oldvol && graph->mincut == oldcut))
                    break;
            }

            ipqDestroy(queue);

            WCOREPOP;
        }

        void ComputeSubDomainGraph(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, ii, j, pid, other, nparts, nvtxs, nnbrs;
            idx_t* where;
            idx_t nads = 0, * vadids, * vadwgts;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            where = graph->where.data();

            nparts = ctrl->nparts;

            vadids = ctrl->pvec1.data();
            vadwgts = iset(nparts, 0, ctrl->pvec2.data());

            std::vector<idx_t> pptr(nparts + 1);
            std::vector<idx_t> pind(nvtxs);

            iarray2csr(nvtxs, nparts, where, pptr.data(), pind.data());

            for (pid = 0; pid < nparts; pid++) {
                switch (ctrl->objtype) {
                case METIS_OBJTYPE_CUT:
                {
                    ckrinfo_t* rinfo;
                    cnbr_t* nbrs;

                    rinfo = graph->ckrinfo.data();
                    for (nads = 0, ii = pptr[pid]; ii < pptr[pid + 1]; ii++) {
                        i = pind[ii];
                        ASSERT(pid == where[i]);

                        if (rinfo[i].ed > 0) {
                            nnbrs = rinfo[i].nnbrs;
                            nbrs = ctrl->cnbrpool.data() + rinfo[i].inbr;

                            for (j = 0; j < nnbrs; j++) {
                                other = nbrs[j].pid;
                                if (vadwgts[other] == 0)
                                    vadids[nads++] = other;
                                vadwgts[other] += nbrs[j].ed;
                            }
                        }
                    }
                }
                break;

                case METIS_OBJTYPE_VOL:
                {
                    vkrinfo_t* rinfo;
                    vnbr_t* nbrs;

                    rinfo = graph->vkrinfo.data();
                    for (nads = 0, ii = pptr[pid]; ii < pptr[pid + 1]; ii++) {
                        i = pind[ii];
                        ASSERT(pid == where[i]);

                        if (rinfo[i].ned > 0) {
                            nnbrs = rinfo[i].nnbrs;
                            nbrs = ctrl->vnbrpool.data() + rinfo[i].inbr;

                            for (j = 0; j < nnbrs; j++) {
                                other = nbrs[j].pid;
                                if (vadwgts[other] == 0)
                                    vadids[nads++] = other;
                                vadwgts[other] += nbrs[j].ned;
                            }
                        }
                    }
                }
                break;

                default:
                    gk_errexit("Unknown objtype");
                }

                /* See if you have enough memory to store the adjacent info for that subdomain */
                if (ctrl->maxnads[pid] < nads) {
                    ctrl->maxnads[pid] = 2 * nads;
                    ctrl->adids[pid].resize(ctrl->maxnads[pid]);
                    ctrl->adwgts[pid].resize(ctrl->maxnads[pid]);
                }

                ctrl->nads[pid] = nads;
                for (j = 0; j < nads; j++) {
                    ctrl->adids[pid][j] = vadids[j];
                    ctrl->adwgts[pid][j] = vadwgts[vadids[j]];

                    vadwgts[vadids[j]] = 0;
                }
            }

            WCOREPOP;
        }

        void Greedy_McKWayVolOptimize(ctrl_t* ctrl, graph_t* graph, idx_t niter,
            real_t ffactor, idx_t omode)
        {
            /* Common variables to all types of kway-refinement/balancing routines */
            idx_t i, ii, iii, j, k, pass, nvtxs, ncon, nparts, gain;
            idx_t from, me, to, cto, oldcut;
            idx_t* xadj, * vwgt, * adjncy;
            idx_t* where, * pwgts, * bndind;
            idx_t nmoved, nupd;
            idx_t maxndoms = 0, * nads = NULL, * doms = NULL;
            std::vector<idx_t>* adids = NULL;
            idx_t* bfslvl = NULL, * bfsind = NULL, * bfsmrk = NULL;
            idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
            real_t* pijbm;
            real_t origbal;

            /* Volume-specific/different variables */
            ipq_t* queue;
            idx_t oldvol, xgain;
            vkrinfo_t* myrinfo;
            vnbr_t* mynbrs;

            WCOREPUSH;

            /* Link the graph fields */
            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            bndind = graph->bndind.data();
            where = graph->where.data();
            pwgts = graph->pwgts.data();

            nparts = ctrl->nparts;
            pijbm = ctrl->pijbm.data();

            /* Determine the ubfactors. The method used is different based on omode.
               When OMODE_BALANCE, the ubfactors are those supplied by the user.
               When OMODE_REFINE, the ubfactors are the max of the current partition
               and the user-specified ones. */
            std::vector<float> ubfactors(ncon);
            ComputeLoadImbalanceVec(graph, nparts, pijbm, ubfactors.data());
            origbal = rvecmaxdiff(ncon, ubfactors.data(), ctrl->ubfactors.data());
            (void)origbal;
            if (omode == OMODE_BALANCE) {
                rcopy(ncon, ctrl->ubfactors.data(), ubfactors.data());
            }
            else {
                for (i = 0; i < ncon; i++)
                    ubfactors[i] = (ubfactors[i] > ctrl->ubfactors[i] ? ubfactors[i] : ctrl->ubfactors[i]);
            }


            /* Setup the weight intervals of the various subdomains */
            std::vector<idx_t> minpwgts(nparts * ncon);
            std::vector<idx_t> maxpwgts(nparts * ncon);

            for (i = 0; i < nparts; i++) {
                for (j = 0; j < ncon; j++) {
                    maxpwgts[i * ncon + j] = (idx_t)(ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * ubfactors[j]);
                    /*minpwgts[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*(.9/ubfactors[j]); */
                    minpwgts[i * ncon + j] = (idx_t)(ctrl->tpwgts[i * ncon + j] * graph->tvwgt[j] * 0.2f);
                }
            }

            std::vector<idx_t> perm(nvtxs);

            /* This stores the valid target subdomains. It is used when ctrl->minconn to
               control the subdomains to which moves are allowed to be made.
               When ctrl->minconn is false, the default values of 2 allow all moves to
               go through and it does not interfere with the zero-gain move selection. */
            std::vector<idx_t> safetos(nparts, 2);

            if (ctrl->minconn) {
                ComputeSubDomainGraph(ctrl, graph);

                nads = ctrl->nads.data();
                adids = ctrl->adids.data();
                doms = iset(nparts, 0, ctrl->pvec1.data());
            }


            /* Setup updptr, updind like boundary info to keep track of the vertices whose
               vstatus's need to be reset at the end of the inner iteration */
            std::vector<idx_t> vstatus(nvtxs, VPQSTATUS_NOTPRESENT);
            std::vector<idx_t> updptr(nvtxs, -1);
            std::vector<idx_t> updind(nvtxs);

            std::vector<idx_t> v_bfslvl, v_bfsind, v_bfsmrk;

            if (ctrl->contig) {
                /* The arrays that will be used for limited check of articulation points */
                v_bfslvl.resize(nvtxs);
                v_bfsind.resize(nvtxs);
                v_bfsmrk.resize(nvtxs);

                bfslvl = iset(nvtxs, 0, v_bfslvl.data());
                bfsind = v_bfsind.data();
                bfsmrk = iset(nvtxs, 0, v_bfsmrk.data());
            }

            /* Vol-refinement specific working arrays */
            std::vector<idx_t> modind(nvtxs);
            std::vector<idx_t> vmarker(nvtxs, 0);
            std::vector<idx_t> pmarker(nparts, -1);

            queue = ipqCreate(nvtxs);

            /*=====================================================================
            * The top-level refinement loop
            *======================================================================*/
            for (pass = 0; pass < niter; pass++) {
                ASSERT(ComputeVolume(graph, where) == graph->minvol);

                /* In balancing mode, exit as soon as balance is reached */
                if (omode == OMODE_BALANCE && IsBalanced(ctrl, graph, 0))
                    break;

                oldcut = graph->mincut;
                oldvol = graph->minvol;
                nupd = 0;

                if (ctrl->minconn)
                    maxndoms = imax(nparts, nads, 1);

                /* Insert the boundary vertices in the priority queue */
                irandArrayPermute(graph->nbnd, perm.data(), graph->nbnd / 4, 1);
                for (ii = 0; ii < graph->nbnd; ii++) {
                    i = bndind[perm[ii]];
                    ipqInsert(queue, i, graph->vkrinfo[i].gv);
                    vstatus[i] = VPQSTATUS_PRESENT;
                    ListInsert(nupd, updind, updptr, i);
                }

                /* Start extracting vertices from the queue and try to move them */
                for (nmoved = 0, iii = 0;; iii++) {
                    if ((i = ipqGetTop(queue)) == -1)
                        break;
                    vstatus[i] = VPQSTATUS_EXTRACTED;

                    myrinfo = graph->vkrinfo.data() + i;
                    mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                    from = where[i];

                    /* Prevent moves that make 'from' domain underbalanced */
                    if (omode == OMODE_REFINE) {
                        if (myrinfo->nid > 0 &&
                            !ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts.data() + from * ncon))
                            continue;
                    }
                    else { /* OMODE_BALANCE */
                        if (!ivecaxpygez(ncon, -1, vwgt + i * ncon, pwgts + from * ncon, minpwgts.data() + from * ncon))
                            continue;
                    }

                    if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
                        continue;

                    if (ctrl->minconn)
                        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

                    xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? graph->vsize[i] : 0);

                    /* Find the most promising subdomain to move to */
                    if (omode == OMODE_REFINE) {
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            gain = mynbrs[k].gv + xgain;
                            if (gain >= 0 && ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon))
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        cto = to;
                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            gain = mynbrs[j].gv + xgain;
                            if ((mynbrs[j].gv > mynbrs[k].gv &&
                                ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon))
                                ||
                                (mynbrs[j].gv == mynbrs[k].gv &&
                                    mynbrs[j].ned > mynbrs[k].ned &&
                                    ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon))
                                ||
                                (mynbrs[j].gv == mynbrs[k].gv &&
                                    mynbrs[j].ned == mynbrs[k].ned &&
                                    BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                        1, pwgts + cto * ncon, pijbm + cto * ncon,
                                        1, pwgts + to * ncon, pijbm + to * ncon))) {
                                k = j;
                                cto = to;
                            }
                        }
                        to = cto;

                        j = 0;
                        if (xgain + mynbrs[k].gv > 0 || mynbrs[k].ned - myrinfo->nid > 0)
                            j = 1;
                        else if (mynbrs[k].ned - myrinfo->nid == 0) {
                            if ((iii % 2 == 0 && safetos[to] == 2) ||
                                BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                    -1, pwgts + from * ncon, pijbm + from * ncon,
                                    +1, pwgts + to * ncon, pijbm + to * ncon))
                                j = 1;
                        }
                        if (j == 0)
                            continue;
                    }
                    else { /* OMODE_BALANCE */
                        for (k = myrinfo->nnbrs - 1; k >= 0; k--) {
                            if (!safetos[to = mynbrs[k].pid])
                                continue;
                            if (ivecaxpylez(ncon, 1, vwgt + i * ncon, pwgts + to * ncon, maxpwgts.data() + to * ncon) ||
                                BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                    -1, pwgts + from * ncon, pijbm + from * ncon,
                                    +1, pwgts + to * ncon, pijbm + to * ncon))
                                break;
                        }
                        if (k < 0)
                            continue;  /* break out if you did not find a candidate */

                        cto = to;
                        for (j = k - 1; j >= 0; j--) {
                            if (!safetos[to = mynbrs[j].pid])
                                continue;
                            if (BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                1, pwgts + cto * ncon, pijbm + cto * ncon,
                                1, pwgts + to * ncon, pijbm + to * ncon)) {
                                k = j;
                                cto = to;
                            }
                        }
                        to = cto;

                        if ((xgain + mynbrs[k].gv < 0 ||
                            (xgain + mynbrs[k].gv == 0 && mynbrs[k].ned - myrinfo->nid < 0))
                            &&
                            !BetterBalanceKWay(ncon, vwgt + i * ncon, ubfactors.data(),
                                -1, pwgts + from * ncon, pijbm + from * ncon,
                                +1, pwgts + to * ncon, pijbm + to * ncon))
                            continue;
                    }


                    /*=====================================================================
                    * If we got here, we can now move the vertex from 'from' to 'to'
                    *======================================================================*/
                    graph->mincut -= mynbrs[k].ned - myrinfo->nid;
                    graph->minvol -= (xgain + mynbrs[k].gv);
                    where[i] = to;
                    nmoved++;

                    /* Update the subdomain connectivity information */
                    if (ctrl->minconn) {
                        /* take care of i's move itself */
                        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->nid - mynbrs[k].ned, &maxndoms);

                        /* take care of the adjacent vertices */
                        for (j = xadj[i]; j < xadj[i + 1]; j++) {
                            me = where[adjncy[j]];
                            if (me != from && me != to) {
                                UpdateEdgeSubDomainGraph(ctrl, from, me, -1, &maxndoms);
                                UpdateEdgeSubDomainGraph(ctrl, to, me, 1, &maxndoms);
                            }
                        }
                    }

                    /* Update pwgts */
                    iaxpy(ncon, 1, vwgt + i * ncon, 1, pwgts + to * ncon, 1);
                    iaxpy(ncon, -1, vwgt + i * ncon, 1, pwgts + from * ncon, 1);

                    /* Update the id/ed/gains/bnd/queue of potentially affected nodes */
                    KWayVolUpdate(ctrl, graph, i, from, to, queue, vstatus.data(), &nupd, updptr.data(),
                        updind.data(), bndtype, vmarker.data(), pmarker.data(), modind.data());

                    /*CheckKWayVolPartitionParams(ctrl, graph); */
                }


                /* Reset the vstatus and associated data structures */
                for (i = 0; i < nupd; i++) {
                    ASSERT(updptr[updind[i]] != -1);
                    ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
                    vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
                    updptr[updind[i]] = -1;
                }

                if (nmoved == 0 ||
                    (omode == OMODE_REFINE && graph->minvol == oldvol && graph->mincut == oldcut))
                    break;
            }

            ipqDestroy(queue);

            WCOREPOP;
        }

        void cnbrpoolReset(ctrl_t* ctrl)
        {
            ctrl->nbrpoolcpos = 0;
        }

        idx_t cnbrpoolGetNext(ctrl_t* ctrl, idx_t nnbrs)
        {
            nnbrs = gk_min(ctrl->nparts, nnbrs);
            ctrl->nbrpoolcpos += nnbrs;

            if (ctrl->nbrpoolcpos > ctrl->nbrpoolsize) {
                ctrl->nbrpoolsize += gk_max((size_t)(10 * nnbrs), ctrl->nbrpoolsize / 2);
                ctrl->nbrpoolsize = gk_min(ctrl->nbrpoolsize, ctrl->nbrpoolsize_max);

                ctrl->cnbrpool.resize(ctrl->nbrpoolsize);
                ctrl->nbrpoolreallocs++;
            }

            return (idx_t)(ctrl->nbrpoolcpos - nnbrs);
        }

        void vnbrpoolReset(ctrl_t* ctrl)
        {
            ctrl->nbrpoolcpos = 0;
        }

        idx_t vnbrpoolGetNext(ctrl_t* ctrl, idx_t nnbrs)
        {
            nnbrs = gk_min(ctrl->nparts, nnbrs);
            ctrl->nbrpoolcpos += nnbrs;

            if (ctrl->nbrpoolcpos > ctrl->nbrpoolsize) {
                ctrl->nbrpoolsize += gk_max((size_t)(10 * nnbrs), ctrl->nbrpoolsize / 2);
                ctrl->nbrpoolsize = gk_min(ctrl->nbrpoolsize, ctrl->nbrpoolsize_max);

                ctrl->vnbrpool.resize(ctrl->nbrpoolsize);
                ctrl->nbrpoolreallocs++;
            }

            return (idx_t)(ctrl->nbrpoolcpos - nnbrs);
        }

        void KWayVolUpdate(ctrl_t* ctrl, graph_t* graph, idx_t v, idx_t from,
            idx_t to, ipq_t* queue, idx_t* vstatus, idx_t* r_nupd, idx_t* updptr,
            idx_t* updind, idx_t bndtype, idx_t* vmarker, idx_t* pmarker,
            idx_t* modind)
        {
            idx_t i, ii, iii, j, jj, k, kk, u, nmod, other, me, myidx;
            idx_t* xadj, * vsize, * adjncy, * where;
            vkrinfo_t* myrinfo, * orinfo;
            vnbr_t* mynbrs, * onbrs;

            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            vsize = graph->vsize.data();
            where = graph->where.data();

            myrinfo = graph->vkrinfo.data() + v;
            mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;


            /*======================================================================
             * Remove the contributions on the gain made by 'v'.
             *=====================================================================*/
            for (k = 0; k < myrinfo->nnbrs; k++)
                pmarker[mynbrs[k].pid] = k;
            pmarker[from] = k;

            myidx = pmarker[to];  /* Keep track of the index in mynbrs of the 'to' domain */

            for (j = xadj[v]; j < xadj[v + 1]; j++) {
                ii = adjncy[j];
                other = where[ii];
                orinfo = graph->vkrinfo.data() + ii;
                onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                if (other == from) {
                    for (k = 0; k < orinfo->nnbrs; k++) {
                        if (pmarker[onbrs[k].pid] == -1)
                            onbrs[k].gv += vsize[v];
                    }
                }
                else {
                    ASSERT(pmarker[other] != -1);

                    if (mynbrs[pmarker[other]].ned > 1) {
                        for (k = 0; k < orinfo->nnbrs; k++) {
                            if (pmarker[onbrs[k].pid] == -1)
                                onbrs[k].gv += vsize[v];
                        }
                    }
                    else { /* There is only one connection */
                        for (k = 0; k < orinfo->nnbrs; k++) {
                            if (pmarker[onbrs[k].pid] != -1)
                                onbrs[k].gv -= vsize[v];
                        }
                    }
                }
            }

            for (k = 0; k < myrinfo->nnbrs; k++)
                pmarker[mynbrs[k].pid] = -1;
            pmarker[from] = -1;


            /*======================================================================
             * Update the id/ed of vertex 'v'
             *=====================================================================*/
            if (myidx == -1) {
                myidx = myrinfo->nnbrs++;
                ASSERT(myidx < xadj[v + 1] - xadj[v]);
                mynbrs[myidx].ned = 0;
            }
            myrinfo->ned += myrinfo->nid - mynbrs[myidx].ned;
            SWAP(myrinfo->nid, mynbrs[myidx].ned, j);
            if (mynbrs[myidx].ned == 0)
                mynbrs[myidx] = mynbrs[--myrinfo->nnbrs];
            else
                mynbrs[myidx].pid = from;


            /*======================================================================
             * Update the degrees of adjacent vertices and their volume gains
             *=====================================================================*/
            vmarker[v] = 1;
            modind[0] = v;
            nmod = 1;
            for (j = xadj[v]; j < xadj[v + 1]; j++) {
                ii = adjncy[j];
                me = where[ii];

                if (!vmarker[ii]) {  /* The marking is done for boundary and max gv calculations */
                    vmarker[ii] = 2;
                    modind[nmod++] = ii;
                }

                myrinfo = graph->vkrinfo.data() + ii;
                if (myrinfo->inbr == -1)
                    myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[ii + 1] - xadj[ii]);
                mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                if (me == from) {
                    INC_DEC(myrinfo->ned, myrinfo->nid, 1);
                }
                else if (me == to) {
                    INC_DEC(myrinfo->nid, myrinfo->ned, 1);
                }

                /* Remove the edgeweight from the 'pid == from' entry of the vertex */
                if (me != from) {
                    for (k = 0; k < myrinfo->nnbrs; k++) {
                        if (mynbrs[k].pid == from) {
                            if (mynbrs[k].ned == 1) {
                                mynbrs[k] = mynbrs[--myrinfo->nnbrs];
                                vmarker[ii] = 1;  /* You do a complete .gv calculation */

                                /* All vertices adjacent to 'ii' need to be updated */
                                for (jj = xadj[ii]; jj < xadj[ii + 1]; jj++) {
                                    u = adjncy[jj];
                                    other = where[u];
                                    orinfo = graph->vkrinfo.data() + u;
                                    onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                                    for (kk = 0; kk < orinfo->nnbrs; kk++) {
                                        if (onbrs[kk].pid == from) {
                                            onbrs[kk].gv -= vsize[ii];
                                            if (!vmarker[u]) { /* Need to update boundary etc */
                                                vmarker[u] = 2;
                                                modind[nmod++] = u;
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                            else {
                                mynbrs[k].ned--;

                                /* Update the gv due to single 'ii' connection to 'from' */
                                if (mynbrs[k].ned == 1) {
                                    /* find the vertex 'u' that 'ii' was connected into 'from' */
                                    for (jj = xadj[ii]; jj < xadj[ii + 1]; jj++) {
                                        u = adjncy[jj];
                                        other = where[u];

                                        if (other == from) {
                                            orinfo = graph->vkrinfo.data() + u;
                                            onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                                            /* The following is correct because domains in common
                                               between ii and u will lead to a reduction over the
                                               previous gain, whereas domains only in u but not in
                                               ii, will lead to no change as opposed to the earlier
                                               increase */
                                            for (kk = 0; kk < orinfo->nnbrs; kk++)
                                                onbrs[kk].gv += vsize[ii];

                                            if (!vmarker[u]) { /* Need to update boundary etc */
                                                vmarker[u] = 2;
                                                modind[nmod++] = u;
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                            break;
                        }
                    }
                }


                /* Add the edgeweight to the 'pid == to' entry of the vertex */
                if (me != to) {
                    for (k = 0; k < myrinfo->nnbrs; k++) {
                        if (mynbrs[k].pid == to) {
                            mynbrs[k].ned++;

                            /* Update the gv due to non-single 'ii' connection to 'to' */
                            if (mynbrs[k].ned == 2) {
                                /* find the vertex 'u' that 'ii' was connected into 'to' */
                                for (jj = xadj[ii]; jj < xadj[ii + 1]; jj++) {
                                    u = adjncy[jj];
                                    other = where[u];

                                    if (u != v && other == to) {
                                        orinfo = graph->vkrinfo.data() + u;
                                        onbrs = ctrl->vnbrpool.data() + orinfo->inbr;
                                        for (kk = 0; kk < orinfo->nnbrs; kk++)
                                            onbrs[kk].gv -= vsize[ii];

                                        if (!vmarker[u]) { /* Need to update boundary etc */
                                            vmarker[u] = 2;
                                            modind[nmod++] = u;
                                        }
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }

                    if (k == myrinfo->nnbrs) {
                        mynbrs[myrinfo->nnbrs].pid = to;
                        mynbrs[myrinfo->nnbrs++].ned = 1;
                        vmarker[ii] = 1;  /* You do a complete .gv calculation */

                        /* All vertices adjacent to 'ii' need to be updated */
                        for (jj = xadj[ii]; jj < xadj[ii + 1]; jj++) {
                            u = adjncy[jj];
                            other = where[u];
                            orinfo = graph->vkrinfo.data() + u;
                            onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                            for (kk = 0; kk < orinfo->nnbrs; kk++) {
                                if (onbrs[kk].pid == to) {
                                    onbrs[kk].gv += vsize[ii];
                                    if (!vmarker[u]) { /* Need to update boundary etc */
                                        vmarker[u] = 2;
                                        modind[nmod++] = u;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }

                ASSERT(myrinfo->nnbrs <= xadj[ii + 1] - xadj[ii]);
            }


            /*======================================================================
             * Add the contributions on the volume gain due to 'v'
             *=====================================================================*/
            myrinfo = graph->vkrinfo.data() + v;
            mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;
            for (k = 0; k < myrinfo->nnbrs; k++)
                pmarker[mynbrs[k].pid] = k;
            pmarker[to] = k;

            for (j = xadj[v]; j < xadj[v + 1]; j++) {
                ii = adjncy[j];
                other = where[ii];
                orinfo = graph->vkrinfo.data() + ii;
                onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                if (other == to) {
                    for (k = 0; k < orinfo->nnbrs; k++) {
                        if (pmarker[onbrs[k].pid] == -1)
                            onbrs[k].gv -= vsize[v];
                    }
                }
                else {
                    ASSERT(pmarker[other] != -1);

                    if (mynbrs[pmarker[other]].ned > 1) {
                        for (k = 0; k < orinfo->nnbrs; k++) {
                            if (pmarker[onbrs[k].pid] == -1)
                                onbrs[k].gv -= vsize[v];
                        }
                    }
                    else { /* There is only one connection */
                        for (k = 0; k < orinfo->nnbrs; k++) {
                            if (pmarker[onbrs[k].pid] != -1)
                                onbrs[k].gv += vsize[v];
                        }
                    }
                }
            }
            for (k = 0; k < myrinfo->nnbrs; k++)
                pmarker[mynbrs[k].pid] = -1;
            pmarker[to] = -1;


            /*======================================================================
             * Recompute the volume information of the 'hard' nodes, and update the
             * max volume gain for all the modified vertices and the priority queue
             *=====================================================================*/
            for (iii = 0; iii < nmod; iii++) {
                i = modind[iii];
                me = where[i];

                myrinfo = graph->vkrinfo.data() + i;
                mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                if (vmarker[i] == 1) {  /* Only complete gain updates go through */
                    for (k = 0; k < myrinfo->nnbrs; k++)
                        mynbrs[k].gv = 0;

                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        ii = adjncy[j];
                        other = where[ii];
                        orinfo = graph->vkrinfo.data() + ii;
                        onbrs = ctrl->vnbrpool.data() + orinfo->inbr;

                        for (kk = 0; kk < orinfo->nnbrs; kk++)
                            pmarker[onbrs[kk].pid] = kk;
                        pmarker[other] = 1;

                        if (me == other) {
                            /* Find which domains 'i' is connected and 'ii' is not and update their gain */
                            for (k = 0; k < myrinfo->nnbrs; k++) {
                                if (pmarker[mynbrs[k].pid] == -1)
                                    mynbrs[k].gv -= vsize[ii];
                            }
                        }
                        else {
                            ASSERT(pmarker[me] != -1);

                            /* I'm the only connection of 'ii' in 'me' */
                            if (onbrs[pmarker[me]].ned == 1) {
                                /* Increase the gains for all the common domains between 'i' and 'ii' */
                                for (k = 0; k < myrinfo->nnbrs; k++) {
                                    if (pmarker[mynbrs[k].pid] != -1)
                                        mynbrs[k].gv += vsize[ii];
                                }
                            }
                            else {
                                /* Find which domains 'i' is connected and 'ii' is not and update their gain */
                                for (k = 0; k < myrinfo->nnbrs; k++) {
                                    if (pmarker[mynbrs[k].pid] == -1)
                                        mynbrs[k].gv -= vsize[ii];
                                }
                            }
                        }

                        for (kk = 0; kk < orinfo->nnbrs; kk++)
                            pmarker[onbrs[kk].pid] = -1;
                        pmarker[other] = -1;

                    }
                }

                /* Compute the overall gv for that node */
                myrinfo->gv = IDX_MIN;
                for (k = 0; k < myrinfo->nnbrs; k++) {
                    if (mynbrs[k].gv > myrinfo->gv)
                        myrinfo->gv = mynbrs[k].gv;
                }

                /* Add the xtra gain due to id == 0 */
                if (myrinfo->ned > 0 && myrinfo->nid == 0)
                    myrinfo->gv += vsize[i];


                /*======================================================================
                 * Maintain a consistent boundary
                 *=====================================================================*/
                if (bndtype == BNDTYPE_REFINE) {
                    if (myrinfo->gv >= 0 && graph->bndptr[i] == -1)
                        BNDInsert(graph->nbnd, graph->bndind, graph->bndptr, i);

                    if (myrinfo->gv < 0 && graph->bndptr[i] != -1)
                        BNDDelete(graph->nbnd, graph->bndind, graph->bndptr, i);
                }
                else {
                    if (myrinfo->ned > 0 && graph->bndptr[i] == -1)
                        BNDInsert(graph->nbnd, graph->bndind, graph->bndptr, i);

                    if (myrinfo->ned == 0 && graph->bndptr[i] != -1)
                        BNDDelete(graph->nbnd, graph->bndind, graph->bndptr, i);
                }


                /*======================================================================
                 * Update the priority queue appropriately (if allowed)
                 *=====================================================================*/
                if (queue != NULL) {
                    if (vstatus[i] != VPQSTATUS_EXTRACTED) {
                        if (graph->bndptr[i] != -1) { /* In-boundary vertex */
                            if (vstatus[i] == VPQSTATUS_PRESENT) {
                                ipqUpdate(queue, i, myrinfo->gv);
                            }
                            else {
                                ipqInsert(queue, i, myrinfo->gv);
                                vstatus[i] = VPQSTATUS_PRESENT;
                                ListInsert(*r_nupd, updind, updptr, i);
                            }
                        }
                        else { /* Off-boundary vertex */
                            if (vstatus[i] == VPQSTATUS_PRESENT) {
                                ipqDelete(queue, i);
                                vstatus[i] = VPQSTATUS_NOTPRESENT;
                                ListDelete(*r_nupd, updind, updptr, i);
                            }
                        }
                    }
                }

                vmarker[i] = 0;
            }
        }


        void UpdateEdgeSubDomainGraph(ctrl_t* ctrl, idx_t u, idx_t v, idx_t ewgt,
            idx_t* r_maxndoms)
        {
            idx_t i, j, nads;

            if (ewgt == 0)
                return;

            for (i = 0; i < 2; i++) {
                nads = ctrl->nads[u];
                /* Find the edge */
                for (j = 0; j < nads; j++) {
                    if (ctrl->adids[u][j] == v) {
                        ctrl->adwgts[u][j] += ewgt;
                        break;
                    }
                }

                if (j == nads) {
                    /* Deal with the case in which the edge was not found */
                    ASSERT(ewgt > 0);
                    if (ctrl->maxnads[u] == nads) {
                        ctrl->maxnads[u] = 2 * (nads + 1);
                        ctrl->adids[u].resize(ctrl->maxnads[u]);
                        ctrl->adwgts[u].resize(ctrl->maxnads[u]);
                    }
                    ctrl->adids[u][nads] = v;
                    ctrl->adwgts[u][nads] = ewgt;
                    nads++;
                    if (r_maxndoms != NULL && nads > *r_maxndoms) {
                        *r_maxndoms = nads;
                    }
                }
                else {
                    /* See if the updated edge becomes 0 */
                    ASSERT(ctrl->adwgts[u][j] >= 0);
                    if (ctrl->adwgts[u][j] == 0) {
                        ctrl->adids[u][j] = ctrl->adids[u][nads - 1];
                        ctrl->adwgts[u][j] = ctrl->adwgts[u][nads - 1];
                        nads--;
                        if (r_maxndoms != NULL && nads + 1 == *r_maxndoms)
                            *r_maxndoms = ctrl->nads[iargmax(ctrl->nparts, ctrl->nads.data(), 1)];
                    }
                }
                ctrl->nads[u] = nads;

                SWAP(u, v, j);
            }
        }


        /*************************************************************************/
        /*! This function is the top-level driver of the recursive bisection
            routine. */
            /*************************************************************************/
        idx_t MlevelRecursiveBisection(ctrl_t* ctrl, graph_t* graph, idx_t nparts,
            idx_t* part, real_t* tpwgts, idx_t fpart)
        {
            idx_t i, nvtxs, ncon, objval;
            idx_t* label, * where;
            graph_t* lgraph = nullptr, * rgraph = nullptr;
            real_t wsum;
            std::vector<real_t> tpwgts2;

            if ((nvtxs = graph->nvtxs) == 0) {
                printf("\t***Cannot bisect a graph with 0 vertices!\n"
                    "\t***You are trying to partition a graph into too many parts!\n");
                return 0;
            }

            ncon = graph->ncon;

            /* determine the weights of the two partitions as a function of the weight of the
               target partition weights */
            WCOREPUSH;
            tpwgts2.resize(2 * ncon);
            for (i = 0; i < ncon; i++) {
                tpwgts2[i] = rsum((nparts >> 1), tpwgts + i, ncon);
                tpwgts2[ncon + i] = 1.0f - tpwgts2[i];
            }

            /* perform the bisection */
            objval = MultilevelBisect(ctrl, graph, tpwgts2.data());

            WCOREPOP;

            label = graph->label.data();
            where = graph->where.data();
            for (i = 0; i < nvtxs; i++)
                part[label[i]] = where[i] + fpart;

            if (nparts > 2)
                SplitGraphPart(ctrl, graph, &lgraph, &rgraph);

            /* Free the memory of the top level graph */
            delete graph;

            /* Scale the fractions in the tpwgts according to the true weight */
            for (i = 0; i < ncon; i++) {
                wsum = rsum((nparts >> 1), tpwgts + i, ncon);
                rscale((nparts >> 1), 1.0f / wsum, tpwgts + i, ncon);
                rscale(nparts - (nparts >> 1), 1.0f / (1.0f - wsum), tpwgts + (nparts >> 1) * ncon + i, ncon);
            }

            /* Do the recursive call */
            if (nparts > 3) {
                objval += MlevelRecursiveBisection(ctrl, lgraph, (nparts >> 1), part,
                    tpwgts, fpart);
                objval += MlevelRecursiveBisection(ctrl, rgraph, nparts - (nparts >> 1), part,
                    tpwgts + (nparts >> 1) * ncon, fpart + (nparts >> 1));
            }
            else if (nparts == 3) {
                delete lgraph;
                objval += MlevelRecursiveBisection(ctrl, rgraph, nparts - (nparts >> 1), part,
                    tpwgts + (nparts >> 1) * ncon, fpart + (nparts >> 1));
            }


            return objval;
        }

    private:

#define UNMATCHEDFOR2HOP  0.10
#define COARSEN_FRACTION	0.85
#define MAKECSR(i, n, a) \
	   do { \
		 for (i=1; i<n; i++) a[i] += a[i-1]; \
		 for (i=n; i>0; i--) a[i] = a[i-1]; \
		 a[0] = 0; \
	   } while(0) 

#define SHIFTCSR(i, n, a) \
	   do { \
		 for (i=n; i>0; i--) a[i] = a[i-1]; \
		 a[0] = 0; \
	   } while(0) 

        void iarray2csr(idx_t n, idx_t range, idx_t* array, idx_t* ptr, idx_t* ind)
        {
            idx_t i;
            for (i = 0; i <= range; i++)
                ptr[i] = 0;
            for (i = 0; i < n; i++)
                ptr[array[i]]++;
            MAKECSR(i, range, ptr);
            for (i = 0; i < n; i++)
                ind[ptr[array[i]]++] = i;
            SHIFTCSR(i, range, ptr);
        }

        void Setup2WayBalMultipliers(ctrl_t* ctrl, graph_t* graph, real_t* tpwgts)
        {
            idx_t i, j;
            for (i = 0; i < 2; i++) {
                for (j = 0; j < graph->ncon; j++)
                    ctrl->pijbm[i * graph->ncon + j] = graph->invtvwgt[j] / tpwgts[i * graph->ncon + j];
            }
        }

        void SetupKWayBalMultipliers(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, j;

            for (i = 0; i < ctrl->nparts; i++) {
                for (j = 0; j < graph->ncon; j++)
                    ctrl->pijbm[i * graph->ncon + j] = graph->invtvwgt[j] / ctrl->tpwgts[i * graph->ncon + j];
            }
        }

        void ReAdjustMemory(ctrl_t* ctrl, graph_t* graph, graph_t* cgraph)
        {
            if (cgraph->nedges > 10000 && cgraph->nedges < 0.9 * graph->nedges) {
                cgraph->adjncy.resize(cgraph->nedges);
                cgraph->adjwgt.resize(cgraph->nedges);
            }
        }

        idx_t FindPartitionInducedComponents(graph_t* graph, idx_t* where,
            idx_t* cptr, idx_t* cind)
        {
            idx_t i, j, k, me = 0, nvtxs, first, last, nleft, ncmps;
            idx_t* xadj, * adjncy;
            std::vector<idx_t> touched, perm, todo;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();

            std::vector<idx_t> v_cptr;
            std::vector<idx_t> v_cind;
            std::vector<idx_t> v_where;

            /* Deal with NULL supplied cptr/cind vectors */
            if (cptr == NULL) {

                v_cptr.resize(nvtxs + 1);
                v_cind.resize(nvtxs);
                cptr = v_cptr.data();
                cind = v_cind.data();
            }

            /* Deal with NULL supplied where vector */
            if (where == NULL) {
                v_where.resize(nvtxs);
                where = v_where.data();
            }

            /* Allocate memory required for the BFS traversal */
            perm.resize(nvtxs);
            todo.resize(nvtxs);
            iincset(nvtxs, 0, perm.data());
            iincset(nvtxs, 0, todo.data());
            touched.resize(nvtxs, 0);

            /* Find the connected componends induced by the partition */
            ncmps = -1;
            first = last = 0;
            nleft = nvtxs;
            while (nleft > 0) {
                if (first == last) { /* Find another starting vertex */
                    cptr[++ncmps] = first;
                    ASSERT(touched[todo[0]] == 0);
                    i = todo[0];
                    cind[last++] = i;
                    touched[i] = 1;
                    me = where[i];
                }

                i = cind[first++];
                k = perm[i];
                j = todo[k] = todo[--nleft];
                perm[j] = k;

                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                    k = adjncy[j];
                    if (where[k] == me && !touched[k]) {
                        cind[last++] = k;
                        touched[k] = 1;
                    }
                }
            }
            cptr[++ncmps] = first;

            return ncmps;
        }

        void EliminateSubDomainEdges(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, ii, j, k, ncon, nparts, scheme, pid_from, pid_to, me, other, nvtxs,
                total, max, avg, totalout, nind = 0, ncand = 0, ncand2, target, target2,
                nadd, bestnadd = 0;
            idx_t min, move;
            idx_t* xadj, * adjncy, * vwgt, * adjwgt, * pwgts, * where,
                * mypmat, * kpmat;
            idx_t* nads;
            std::vector<idx_t>* adids, * adwgts;
            real_t* tpwgts, badfactor = 1.4f;
            idx_t* vmarker = NULL, * pmarker = NULL, * modind = NULL;  /* volume specific work arrays */

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            vwgt = graph->vwgt.data();
            adjwgt = (ctrl->objtype == METIS_OBJTYPE_VOL ? NULL : graph->adjwgt.data());

            where = graph->where.data();
            pwgts = graph->pwgts.data();  /* We assume that this is properly initialized */

            nparts = ctrl->nparts;
            tpwgts = ctrl->tpwgts.data();

            std::vector<idx_t> cpwgt(ncon);
            std::vector<idx_t> maxpwgt(nparts * ncon);
            std::vector<idx_t> ind(nvtxs);
            std::vector<idx_t> otherpmat(nparts, 0);
            std::vector<ikv_t> cand(nparts);
            std::vector<ikv_t> cand2(nparts);

            std::vector<idx_t> pptr(nparts + 1);
            std::vector<idx_t> pind(nvtxs);

            iarray2csr(nvtxs, nparts, where, pptr.data(), pind.data());

            std::vector<idx_t> v_modind;
            std::vector<idx_t> v_vmarker;
            std::vector<idx_t> v_pmarker;


            if (ctrl->objtype == METIS_OBJTYPE_VOL) {
                /* Vol-refinement specific working arrays */
                v_modind.resize(nvtxs);
                v_vmarker.resize(nvtxs);
                v_pmarker.resize(nparts);

                modind = v_modind.data();
                vmarker = iset(nvtxs, 0, v_vmarker.data());
                pmarker = iset(nparts, -1, v_pmarker.data());
            }

            /* Compute the pmat matrix and ndoms */
            ComputeSubDomainGraph(ctrl, graph);

            nads = ctrl->nads.data();
            adids = ctrl->adids.data();
            adwgts = ctrl->adwgts.data();

            mypmat = iset(nparts, 0, ctrl->pvec1.data());
            kpmat = iset(nparts, 0, ctrl->pvec2.data());

            /* Compute the maximum allowed weight for each domain */
            for (i = 0; i < nparts; i++) {
                for (j = 0; j < ncon; j++)
                    maxpwgt[i * ncon + j] = (idx_t)(
                        (ncon == 1 ? 1.25f : 1.025f) * tpwgts[i] * graph->tvwgt[j] * ctrl->ubfactors[j]);
            }

            ipq_t queue(nparts);

            /* Get into the loop eliminating subdomain connections */
            while (1) {
                total = isum(nparts, nads, 1);
                avg = total / nparts;
                max = nads[iargmax(nparts, nads, 1)];

                if (max < badfactor * avg)
                    break;

                /* Add the subdomains that you will try to reduce their connectivity */
                ipqReset(&queue);
                for (i = 0; i < nparts; i++) {
                    if (nads[i] >= avg + (max - avg) / 2)
                        ipqInsert(&queue, i, nads[i]);
                }

                move = 0;
                while ((me = ipqGetTop(&queue)) != -1) {
                    totalout = isum(nads[me], adwgts[me].data(), 1);

                    for (ncand2 = 0, i = 0; i < nads[me]; i++) {
                        mypmat[adids[me][i]] = adwgts[me][i];

                        /* keep track of the weakly connected adjacent subdomains */
                        if (2 * nads[me] * adwgts[me][i] < totalout) {
                            cand2[ncand2].val = adids[me][i];
                            cand2[ncand2++].key = adwgts[me][i];
                        }
                    }

                    /* Sort the connections according to their cut */
                    ikvsorti(ncand2, cand2.data());

                    /* Two schemes are used for eliminating subdomain edges.
                       The first, tries to eliminate subdomain edges by moving remote groups
                       of vertices to subdomains that 'me' is already connected to.
                       The second, tries to eliminate subdomain edges by moving entire sets of
                       my vertices that connect to the 'other' subdomain to a subdomain that
                       I'm already connected to.
                       These two schemes are applied in sequence. */
                    target = target2 = -1;
                    for (scheme = 0; scheme < 2; scheme++) {
                        for (min = 0; min < ncand2; min++) {
                            other = cand2[min].val;

                            /* pid_from is the subdomain from where the vertices will be removed.
                               pid_to is the adjacent subdomain to pid_from that defines the
                               (me, other) subdomain edge that needs to be removed */
                            if (scheme == 0) {
                                pid_from = other;
                                pid_to = me;
                            }
                            else {
                                pid_from = me;
                                pid_to = other;
                            }

                            /* Go and find the vertices in 'other' that are connected in 'me' */
                            for (nind = 0, ii = pptr[pid_from]; ii < pptr[pid_from + 1]; ii++) {
                                i = pind[ii];
                                ASSERT(where[i] == pid_from);
                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    if (where[adjncy[j]] == pid_to) {
                                        ind[nind++] = i;
                                        break;
                                    }
                                }
                            }

                            /* Go and construct the otherpmat to see where these nind vertices are
                               connected to */
                            iset(ncon, 0, cpwgt.data());
                            for (ncand = 0, ii = 0; ii < nind; ii++) {
                                i = ind[ii];
                                iaxpy(ncon, 1, vwgt + i * ncon, 1, cpwgt.data(), 1);

                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    if ((k = where[adjncy[j]]) == pid_from)
                                        continue;
                                    if (otherpmat[k] == 0)
                                        cand[ncand++].val = k;
                                    otherpmat[k] += (adjwgt ? adjwgt[j] : 1);
                                }
                            }

                            for (i = 0; i < ncand; i++) {
                                cand[i].key = otherpmat[cand[i].val];
                                ASSERT(cand[i].key > 0);
                            }

                            ikvsortd(ncand, cand.data());

                            /* Go through and select the first domain that is common with 'me', and does
                               not increase the nads[target] higher than nads[me], subject to the maxpwgt
                               constraint. Traversal is done from the mostly connected to the least. */
                            for (i = 0; i < ncand; i++) {
                                k = cand[i].val;

                                if (mypmat[k] > 0) {
                                    /* Check if balance will go off */
                                    if (!ivecaxpylez(ncon, 1, cpwgt.data(), pwgts + k * ncon, maxpwgt.data() + k * ncon))
                                        continue;

                                    /* get a dense vector out of k's connectivity */
                                    for (j = 0; j < nads[k]; j++)
                                        kpmat[adids[k][j]] = adwgts[k][j];

                                    /* Check if the move to domain k will increase the nads of another
                                       subdomain j that the set of vertices being moved are connected
                                       to but domain k is not connected to. */
                                    for (j = 0; j < nparts; j++) {
                                        if (otherpmat[j] > 0 && kpmat[j] == 0 && nads[j] + 1 >= nads[me])
                                            break;
                                    }

                                    /* There were no bad second level effects. See if you can find a
                                       subdomain to move to. */
                                    if (j == nparts) {
                                        for (nadd = 0, j = 0; j < nparts; j++) {
                                            if (otherpmat[j] > 0 && kpmat[j] == 0)
                                                nadd++;
                                        }

                                        if (nads[k] + nadd < nads[me]) {
                                            if (target2 == -1 || nads[target2] + bestnadd > nads[k] + nadd ||
                                                (nads[target2] + bestnadd == nads[k] + nadd && bestnadd > nadd)) {
                                                target2 = k;
                                                bestnadd = nadd;
                                            }
                                        }

                                        if (nadd == 0)
                                            target = k;
                                    }

                                    /* reset kpmat for the next iteration */
                                    for (j = 0; j < nads[k]; j++)
                                        kpmat[adids[k][j]] = 0;
                                }

                                if (target != -1)
                                    break;
                            }

                            /* reset the otherpmat for the next iteration */
                            for (i = 0; i < ncand; i++)
                                otherpmat[cand[i].val] = 0;

                            if (target == -1 && target2 != -1)
                                target = target2;

                            if (target != -1) {
                                move = 1;
                                break;
                            }
                        }

                        if (target != -1)
                            break;  /* A move was found. No need to try the other scheme */
                    }

                    /* reset the mypmat for next iteration */
                    for (i = 0; i < nads[me]; i++)
                        mypmat[adids[me][i]] = 0;

                    /* Note that once a target is found the above loops exit right away. So the
                       following variables are valid */
                    if (target != -1) {
                        switch (ctrl->objtype) {
                        case METIS_OBJTYPE_CUT:
                            MoveGroupMinConnForCut(ctrl, graph, target, nind, ind.data());
                            break;
                        case METIS_OBJTYPE_VOL:
                            MoveGroupMinConnForVol(ctrl, graph, target, nind, ind.data(), vmarker,
                                pmarker, modind);
                            break;
                        default:
                            gk_errexit("Unknown objtype of %d\n");
                        }

                        /* Update the csr representation of the partitioning vector */
                        iarray2csr(nvtxs, nparts, where, pptr.data(), pind.data());
                    }
                }

                if (move == 0)
                    break;
            }

            WCOREPOP;
        }

        void EliminateComponents(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, ii, j, jj, me, nparts, nvtxs, ncon, ncmps,
                ncand, target;
            idx_t* xadj, * adjncy, * vwgt, * adjwgt, * where, * pwgts;
            idx_t cid, bestcid;
            idx_t ntodo, oldntodo;
            idx_t* vmarker = NULL, * pmarker = NULL, * modind = NULL;  /* volume specific work arrays */

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            vwgt = graph->vwgt.data();
            adjwgt = (ctrl->objtype == METIS_OBJTYPE_VOL ? NULL : graph->adjwgt.data());

            where = graph->where.data();
            pwgts = graph->pwgts.data();

            nparts = ctrl->nparts;

            std::vector<idx_t> cptr(nvtxs + 1);
            std::vector<idx_t> cind(nvtxs);

            ncmps = FindPartitionInducedComponents(graph, where, cptr.data(), cind.data());

            /* There are more components than partitions */
            if (ncmps > nparts) {
                std::vector<idx_t> cwgt(ncon);
                std::vector<idx_t> bestcwgt(ncon);
                std::vector<idx_t> cpvec(nparts);
                std::vector<idx_t> pcptr(nparts + 1, 0);
                std::vector<idx_t> pcind(ncmps);
                std::vector<idx_t> cwhere(nvtxs, -1);

                std::vector<idx_t> todo(ncmps);
                std::vector<rkv_t> cand(nparts);

                std::vector<idx_t> v_modind;
                std::vector<idx_t> v_vmarker;
                std::vector<idx_t> v_pmarker;

                if (ctrl->objtype == METIS_OBJTYPE_VOL) {
                    /* Vol-refinement specific working arrays */
                    v_modind.resize(nvtxs);
                    v_vmarker.resize(nvtxs);
                    v_pmarker.resize(nparts);

                    modind = v_modind.data();
                    vmarker = iset(nvtxs, 0, v_vmarker.data());
                    pmarker = iset(nparts, -1, v_pmarker.data());
                }


                /* Get a CSR representation of the components-2-partitions mapping */
                for (i = 0; i < ncmps; i++)
                    pcptr[where[cind[cptr[i]]]]++;
                MAKECSR(i, nparts, pcptr);
                for (i = 0; i < ncmps; i++)
                    pcind[pcptr[where[cind[cptr[i]]]]++] = i;
                SHIFTCSR(i, nparts, pcptr);

                /* Assign the heaviest component of each partition to its original partition */
                for (ntodo = 0, i = 0; i < nparts; i++) {
                    if (pcptr[i + 1] - pcptr[i] == 1)
                        bestcid = pcind[pcptr[i]];
                    else {
                        for (bestcid = -1, j = pcptr[i]; j < pcptr[i + 1]; j++) {
                            cid = pcind[j];
                            iset(ncon, 0, cwgt.data());
                            for (ii = cptr[cid]; ii < cptr[cid + 1]; ii++)
                                iaxpy(ncon, 1, vwgt + cind[ii] * ncon, 1, cwgt.data(), 1);
                            if (bestcid == -1 || isum(ncon, bestcwgt.data(), 1) < isum(ncon, cwgt.data(), 1)) {
                                bestcid = cid;
                                icopy(ncon, cwgt.data(), bestcwgt.data());
                            }
                        }
                        /* Keep track of those that need to be dealt with */
                        for (j = pcptr[i]; j < pcptr[i + 1]; j++) {
                            if (pcind[j] != bestcid)
                                todo[ntodo++] = pcind[j];
                        }
                    }

                    for (j = cptr[bestcid]; j < cptr[bestcid + 1]; j++) {
                        ASSERT(where[cind[j]] == i);
                        cwhere[cind[j]] = i;
                    }
                }


                while (ntodo > 0) {
                    oldntodo = ntodo;
                    for (i = 0; i < ntodo; i++) {
                        cid = todo[i];
                        me = where[cind[cptr[cid]]];  /* Get the domain of this component */

                        /* Determine the weight of the block to be moved */
                        iset(ncon, 0, cwgt.data());
                        for (j = cptr[cid]; j < cptr[cid + 1]; j++)
                            iaxpy(ncon, 1, vwgt + cind[j] * ncon, 1, cwgt.data(), 1);

                        /* Determine the connectivity */
                        iset(nparts, 0, cpvec.data());
                        for (j = cptr[cid]; j < cptr[cid + 1]; j++) {
                            ii = cind[j];
                            for (jj = xadj[ii]; jj < xadj[ii + 1]; jj++)
                                if (cwhere[adjncy[jj]] != -1)
                                    cpvec[cwhere[adjncy[jj]]] += (adjwgt ? adjwgt[jj] : 1);
                        }

                        /* Put the neighbors into a cand[] array for sorting */
                        for (ncand = 0, j = 0; j < nparts; j++) {
                            if (cpvec[j] > 0) {
                                cand[ncand].key = (float)cpvec[j];
                                cand[ncand++].val = j;
                            }
                        }
                        if (ncand == 0)
                            continue;

                        rkvsortd(ncand, cand.data());

                        /* Limit the moves to only the top candidates, which are defined as
                           those with connectivity at least 50% of the best.
                           This applies only when ncon=1, as for multi-constraint, balancing
                           will be hard. */
                        if (ncon == 1) {
                            for (j = 1; j < ncand; j++) {
                                if (cand[j].key < .5 * cand[0].key)
                                    break;
                            }
                            ncand = j;
                        }

                        /* Now among those, select the one with the best balance */
                        target = cand[0].val;
                        for (j = 1; j < ncand; j++) {
                            if (BetterBalanceKWay(ncon, cwgt.data(), ctrl->ubfactors.data(),
                                1, pwgts + target * ncon, ctrl->pijbm.data() + target * ncon,
                                1, pwgts + cand[j].val * ncon, ctrl->pijbm.data() + cand[j].val * ncon))
                                target = cand[j].val;
                        }

                        /* Note that as a result of a previous movement, a connected component may
                           now will like to stay to its original partition */
                        if (target != me) {
                            switch (ctrl->objtype) {
                            case METIS_OBJTYPE_CUT:
                                MoveGroupContigForCut(ctrl, graph, target, cid, cptr.data(), cind.data());
                                break;

                            case METIS_OBJTYPE_VOL:
                                MoveGroupContigForVol(ctrl, graph, target, cid, cptr.data(), cind.data(),
                                    vmarker, pmarker, modind);
                                break;

                            default:
                                gk_errexit("Unknown objtype %d\n");
                            }
                        }

                        /* Update the cwhere vector */
                        for (j = cptr[cid]; j < cptr[cid + 1]; j++)
                            cwhere[cind[j]] = target;

                        todo[i] = todo[--ntodo];
                    }
                    if (oldntodo == ntodo) {
                        break;
                    }
                }

                for (i = 0; i < nvtxs; i++)
                    ASSERT(where[i] == cwhere[i]);

            }

            WCOREPOP;
        }

        void MoveGroupContigForCut(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t gid,
            idx_t* ptr, idx_t* ind)
        {
            idx_t i, ii, iii, j, k, nbnd, from, me;
            idx_t* xadj, * adjncy, * adjwgt, * where, * bndptr, * bndind;
            ckrinfo_t* myrinfo;
            cnbr_t* mynbrs;

            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            where = graph->where.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            nbnd = graph->nbnd;

            for (iii = ptr[gid]; iii < ptr[gid + 1]; iii++) {
                i = ind[iii];
                from = where[i];

                myrinfo = graph->ckrinfo.data() + i;
                if (myrinfo->inbr == -1) {
                    myrinfo->inbr = cnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                    myrinfo->nnbrs = 0;
                }
                mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr;

                /* find the location of 'to' in myrinfo or create it if it is not there */
                for (k = 0; k < myrinfo->nnbrs; k++) {
                    if (mynbrs[k].pid == to)
                        break;
                }
                if (k == myrinfo->nnbrs) {
                    mynbrs[k].pid = to;
                    mynbrs[k].ed = 0;
                    myrinfo->nnbrs++;
                }

                graph->mincut -= mynbrs[k].ed - myrinfo->id;

                /* Update ID/ED and BND related information for the moved vertex */
                iaxpy(graph->ncon, 1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + to * graph->ncon, 1);
                iaxpy(graph->ncon, -1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + from * graph->ncon, 1);
                UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, nbnd,
                    bndptr, bndind, BNDTYPE_REFINE);

                /* Update the degrees of adjacent vertices */
                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                    ii = adjncy[j];
                    me = where[ii];
                    myrinfo = graph->ckrinfo.data() + ii;

                    UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii + 1] - xadj[ii], me,
                        from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, BNDTYPE_REFINE);
                }

                ASSERT(CheckRInfo(ctrl, graph->ckrinfo.data() + i));
            }

            graph->nbnd = nbnd;
        }

        void MoveGroupContigForVol(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t gid,
            idx_t* ptr, idx_t* ind, idx_t* vmarker, idx_t* pmarker,
            idx_t* modind)
        {
            idx_t i, ii, iii, j, k, l, from, other, xgain;
            idx_t* xadj, * vsize, * adjncy, * where;
            vkrinfo_t* myrinfo, * orinfo;
            vnbr_t* mynbrs, * onbrs;

            xadj = graph->xadj.data();
            vsize = graph->vsize.data();
            adjncy = graph->adjncy.data();
            where = graph->where.data();

            for (iii = ptr[gid]; iii < ptr[gid + 1]; iii++) {
                i = ind[iii];
                from = where[i];

                myrinfo = graph->vkrinfo.data() + i;
                if (myrinfo->inbr == -1) {
                    myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                    myrinfo->nnbrs = 0;
                }
                mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? vsize[i] : 0);

                /* find the location of 'to' in myrinfo or create it if it is not there */
                for (k = 0; k < myrinfo->nnbrs; k++) {
                    if (mynbrs[k].pid == to)
                        break;
                }
                if (k == myrinfo->nnbrs) {
                    if (myrinfo->nid > 0)
                        xgain -= vsize[i];

                    /* determine the volume gain resulting from that move */
                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        ii = adjncy[j];
                        other = where[ii];
                        orinfo = graph->vkrinfo.data() + ii;
                        onbrs = ctrl->vnbrpool.data() + orinfo->inbr;
                        ASSERT(other != to)

                            if (from == other) {
                                /* Same subdomain vertex: Decrease the gain if 'to' is a new neighbor. */
                                for (l = 0; l < orinfo->nnbrs; l++) {
                                    if (onbrs[l].pid == to)
                                        break;
                                }
                                if (l == orinfo->nnbrs)
                                    xgain -= vsize[ii];
                            }
                            else {
                                /* Remote vertex: increase if 'to' is a new subdomain */
                                for (l = 0; l < orinfo->nnbrs; l++) {
                                    if (onbrs[l].pid == to)
                                        break;
                                }
                                if (l == orinfo->nnbrs)
                                    xgain -= vsize[ii];

                                /* Remote vertex: decrease if i is the only connection to 'from' */
                                for (l = 0; l < orinfo->nnbrs; l++) {
                                    if (onbrs[l].pid == from && onbrs[l].ned == 1) {
                                        xgain += vsize[ii];
                                        break;
                                    }
                                }
                            }
                    }
                    graph->minvol -= xgain;
                    graph->mincut -= -myrinfo->nid;
                }
                else {
                    graph->minvol -= (xgain + mynbrs[k].gv);
                    graph->mincut -= mynbrs[k].ned - myrinfo->nid;
                }


                /* Update where and pwgts */
                where[i] = to;
                iaxpy(graph->ncon, 1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + to * graph->ncon, 1);
                iaxpy(graph->ncon, -1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + from * graph->ncon, 1);

                /* Update the id/ed/gains/bnd of potentially affected nodes */
                KWayVolUpdate(ctrl, graph, i, from, to, NULL, NULL, NULL, NULL,
                    NULL, BNDTYPE_REFINE, vmarker, pmarker, modind);

                /*CheckKWayVolPartitionParams(ctrl, graph);*/
            }

            ASSERT(ComputeCut(graph, where) == graph->mincut);
            ASSERT(ComputeVolume(graph, where) == graph->minvol);
        }

        /*************************************************************************/
    /*! This function moves a collection of vertices and updates their rinfo */
    /*************************************************************************/
        void MoveGroupMinConnForCut(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t nind,
            idx_t* ind)
        {
            idx_t i, ii, j, k, nbnd, from, me;
            idx_t* xadj, * adjncy, * adjwgt, * where, * bndptr, * bndind;
            ckrinfo_t* myrinfo;
            cnbr_t* mynbrs;

            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            where = graph->where.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            nbnd = graph->nbnd;

            while (--nind >= 0) {
                i = ind[nind];
                from = where[i];

                myrinfo = graph->ckrinfo.data() + i;
                if (myrinfo->inbr == -1) {
                    myrinfo->inbr = cnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                    myrinfo->nnbrs = 0;
                }
                mynbrs = ctrl->cnbrpool.data() + myrinfo->inbr;

                /* find the location of 'to' in myrinfo or create it if it is not there */
                for (k = 0; k < myrinfo->nnbrs; k++) {
                    if (mynbrs[k].pid == to)
                        break;
                }
                if (k == myrinfo->nnbrs) {
                    ASSERT(k < xadj[i + 1] - xadj[i]);
                    mynbrs[k].pid = to;
                    mynbrs[k].ed = 0;
                    myrinfo->nnbrs++;
                }

                /* Update pwgts */
                iaxpy(graph->ncon, 1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + to * graph->ncon, 1);
                iaxpy(graph->ncon, -1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + from * graph->ncon, 1);

                /* Update mincut */
                graph->mincut -= mynbrs[k].ed - myrinfo->id;

                /* Update subdomain connectivity graph to reflect the move of 'i' */
                UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id - mynbrs[k].ed, NULL);

                /* Update ID/ED and BND related information for the moved vertex */
                UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, nbnd,
                    bndptr, bndind, BNDTYPE_REFINE);

                /* Update the degrees of adjacent vertices */
                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                    ii = adjncy[j];
                    me = where[ii];
                    myrinfo = graph->ckrinfo.data() + ii;

                    UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii + 1] - xadj[ii], me,
                        from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, BNDTYPE_REFINE);

                    /* Update subdomain graph to reflect the move of 'i' for domains other
                       than 'from' and 'to' */
                    if (me != from && me != to) {
                        UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], NULL);
                        UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], NULL);
                    }
                }
            }

            ASSERT(ComputeCut(graph, where) == graph->mincut);

            graph->nbnd = nbnd;

        }


        /*************************************************************************/
        /*! This function moves a collection of vertices and updates their rinfo */
        /*************************************************************************/
        void MoveGroupMinConnForVol(ctrl_t* ctrl, graph_t* graph, idx_t to, idx_t nind,
            idx_t* ind, idx_t* vmarker, idx_t* pmarker, idx_t* modind)
        {
            idx_t i, ii, j, k, l, from, me, other, xgain, ewgt;
            idx_t* xadj, * vsize, * adjncy, * where;
            vkrinfo_t* myrinfo, * orinfo;
            vnbr_t* mynbrs, * onbrs;

            xadj = graph->xadj.data();
            vsize = graph->vsize.data();
            adjncy = graph->adjncy.data();
            where = graph->where.data();

            while (--nind >= 0) {
                i = ind[nind];
                from = where[i];

                myrinfo = graph->vkrinfo.data() + i;
                if (myrinfo->inbr == -1) {
                    myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[i + 1] - xadj[i]);
                    myrinfo->nnbrs = 0;
                }
                mynbrs = ctrl->vnbrpool.data() + myrinfo->inbr;

                xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? vsize[i] : 0);

                //printf("Moving %"PRIDX" from %"PRIDX" to %"PRIDX" [vsize: %"PRIDX"] [xgain: %"PRIDX"]\n", 
                //    i, from, to, vsize[i], xgain);

                /* find the location of 'to' in myrinfo or create it if it is not there */
                for (k = 0; k < myrinfo->nnbrs; k++) {
                    if (mynbrs[k].pid == to)
                        break;
                }

                if (k == myrinfo->nnbrs) {
                    //printf("Missing neighbor\n");

                    if (myrinfo->nid > 0)
                        xgain -= vsize[i];

                    /* determine the volume gain resulting from that move */
                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        ii = adjncy[j];
                        other = where[ii];
                        orinfo = graph->vkrinfo.data() + ii;
                        onbrs = ctrl->vnbrpool.data() + orinfo->inbr;
                        ASSERT(other != to)

                            //printf("  %8d %8d %3d\n", (int)ii, (int)vsize[ii], (int)other);

                            if (from == other) {
                                /* Same subdomain vertex: Decrease the gain if 'to' is a new neighbor. */
                                for (l = 0; l < orinfo->nnbrs; l++) {
                                    if (onbrs[l].pid == to)
                                        break;
                                }
                                if (l == orinfo->nnbrs)
                                    xgain -= vsize[ii];
                            }
                            else {
                                /* Remote vertex: increase if 'to' is a new subdomain */
                                for (l = 0; l < orinfo->nnbrs; l++) {
                                    if (onbrs[l].pid == to)
                                        break;
                                }
                                if (l == orinfo->nnbrs)
                                    xgain -= vsize[ii];

                                /* Remote vertex: decrease if i is the only connection to 'from' */
                                for (l = 0; l < orinfo->nnbrs; l++) {
                                    if (onbrs[l].pid == from && onbrs[l].ned == 1) {
                                        xgain += vsize[ii];
                                        break;
                                    }
                                }
                            }
                    }
                    graph->minvol -= xgain;
                    graph->mincut -= -myrinfo->nid;
                    ewgt = myrinfo->nid;
                }
                else {
                    graph->minvol -= (xgain + mynbrs[k].gv);
                    graph->mincut -= mynbrs[k].ned - myrinfo->nid;
                    ewgt = myrinfo->nid - mynbrs[k].ned;
                }

                /* Update where and pwgts */
                where[i] = to;
                iaxpy(graph->ncon, 1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + to * graph->ncon, 1);
                iaxpy(graph->ncon, -1, graph->vwgt.data() + i * graph->ncon, 1, graph->pwgts.data() + from * graph->ncon, 1);

                /* Update subdomain connectivity graph to reflect the move of 'i' */
                UpdateEdgeSubDomainGraph(ctrl, from, to, ewgt, NULL);

                /* Update the subdomain connectivity of the adjacent vertices */
                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                    me = where[adjncy[j]];
                    if (me != from && me != to) {
                        UpdateEdgeSubDomainGraph(ctrl, from, me, -1, NULL);
                        UpdateEdgeSubDomainGraph(ctrl, to, me, 1, NULL);
                    }
                }

                /* Update the id/ed/gains/bnd of potentially affected nodes */
                KWayVolUpdate(ctrl, graph, i, from, to, NULL, NULL, NULL, NULL,
                    NULL, BNDTYPE_REFINE, vmarker, pmarker, modind);

                /*CheckKWayVolPartitionParams(ctrl, graph);*/
            }
            ASSERT(ComputeCut(graph, where) == graph->mincut);
            ASSERT(ComputeVolume(graph, where) == graph->minvol);

        }

        idx_t IsConnected(graph_t* graph, idx_t report)
        {
            idx_t ncmps;

            ncmps = FindPartitionInducedComponents(graph, NULL, NULL, NULL);

            if (ncmps != 1 && report)
            {
                ;
            }

            return (ncmps == 1);
        }

        void BucketSortKeysInc(ctrl_t* ctrl, idx_t n, idx_t max, idx_t* keys,
            idx_t* tperm, idx_t* perm)
        {
            idx_t i, ii;
            idx_t* counts;

            WCOREPUSH;

            std::vector<idx_t> buffer(max + 2);
            counts = iset(max + 2, 0, buffer.data());

            for (i = 0; i < n; i++)
                counts[keys[i]]++;
            MAKECSR(i, max + 1, counts);

            for (ii = 0; ii < n; ii++) {
                i = tperm[ii];
                perm[counts[keys[i]]++] = i;
            }

            WCOREPOP;
        }

        graph_t* SetupCoarseGraph(graph_t* graph, idx_t cnvtxs, int dovsize)
        {
            graph_t* cgraph;

            cgraph = CreateGraph();

            cgraph->nvtxs = cnvtxs;
            cgraph->ncon = graph->ncon;

            cgraph->finer = graph;
            graph->coarser = cgraph;

            cgraph->xadj.resize(cnvtxs + 1);
            cgraph->adjncy.resize(graph->nedges + 1);
            cgraph->adjwgt.resize(graph->nedges + 1);
            cgraph->vwgt.resize(cgraph->ncon * cnvtxs);
            cgraph->tvwgt.resize(cgraph->ncon);
            cgraph->invtvwgt.resize(cgraph->ncon);

            if (dovsize)
                cgraph->vsize.resize(cnvtxs);

            return cgraph;
        }

        void CreateCoarseGraph(ctrl_t* ctrl, graph_t* graph, idx_t cnvtxs,
            idx_t* match)
        {
            idx_t j, k, kk, m, istart, iend, nvtxs, nedges, ncon,
                cnedges, v, u, mask;
            idx_t* xadj, * vwgt, * vsize, * adjncy, * adjwgt;
            idx_t* cmap;
            std::vector<idx_t> htable, dtable;
            idx_t* cxadj, * cvwgt, * cvsize, * cadjncy, * cadjwgt;
            graph_t* cgraph;
            int dovsize, dropedges;
            idx_t nkeys, droppedewgt;
            idx_t* keys = NULL, * medianewgts = NULL, * noise = NULL;

            WCOREPUSH;

            dovsize = (ctrl->objtype == METIS_OBJTYPE_VOL ? 1 : 0);
            dropedges = ctrl->dropedges;

            mask = HTLENGTH;

            //IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ContractTmr));

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            vsize = graph->vsize.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            cmap = graph->cmap.data();

            std::vector<idx_t> v_keys;
            std::vector<idx_t> v_noise;
            std::vector<idx_t> v_medianewgts;

            /* Setup structures for dropedges */
            if (dropedges) {
                for (nkeys = 0, v = 0; v < nvtxs; v++)
                    nkeys = gk_max(nkeys, xadj[v + 1] - xadj[v]);
                nkeys = 2 * nkeys + 1;

                v_keys.resize(nkeys);
                v_noise.resize(cnvtxs);
                v_medianewgts.resize(cnvtxs, -1);

                keys = v_keys.data();
                noise = v_noise.data();
                medianewgts = v_medianewgts.data();

                for (v = 0; v < cnvtxs; v++)
                    noise[v] = (idx_t)irandInRange(128);
            }

            /* Initialize the coarser graph */
            cgraph = SetupCoarseGraph(graph, cnvtxs, dovsize);
            cxadj = cgraph->xadj.data();
            cvwgt = cgraph->vwgt.data();
            cvsize = cgraph->vsize.data();
            cadjncy = cgraph->adjncy.data();
            cadjwgt = cgraph->adjwgt.data();

            htable.resize(mask + 1, -1);   /* hash table */
            dtable.resize(cnvtxs, -1);   /* direct table */

            cxadj[0] = cnvtxs = cnedges = 0;
            for (v = 0; v < nvtxs; v++) {
                if ((u = match[v]) < v)
                    continue;

                ASSERT(cmap[v] == cnvtxs);
                ASSERT(cmap[match[v]] == cnvtxs);

                /* take care of the vertices */
                if (ncon == 1)
                    cvwgt[cnvtxs] = vwgt[v];
                else
                    icopy(ncon, vwgt + v * ncon, cvwgt + cnvtxs * ncon);

                if (dovsize)
                    cvsize[cnvtxs] = vsize[v];

                if (v != u) {
                    if (ncon == 1)
                        cvwgt[cnvtxs] += vwgt[u];
                    else
                        iaxpy(ncon, 1, vwgt + u * ncon, 1, cvwgt + cnvtxs * ncon, 1);

                    if (dovsize)
                        cvsize[cnvtxs] += vsize[u];
                }


                /* take care of the edges */
                if ((xadj[v + 1] - xadj[v] + xadj[u + 1] - xadj[u]) < (mask >> 2)) { /* use mask */
                  /* put the ID of the contracted node itself at the start, so that it can be
                   * removed easily */
                    htable[cnvtxs & mask] = 0;
                    cadjncy[0] = cnvtxs;
                    nedges = 1;

                    istart = xadj[v];
                    iend = xadj[v + 1];
                    for (j = istart; j < iend; j++) {
                        k = cmap[adjncy[j]];
                        for (kk = k & mask; htable[kk] != -1 && cadjncy[htable[kk]] != k; kk = ((kk + 1) & mask));
                        if ((m = htable[kk]) == -1) {
                            cadjncy[nedges] = k;
                            cadjwgt[nedges] = adjwgt[j];
                            htable[kk] = nedges++;
                        }
                        else {
                            cadjwgt[m] += adjwgt[j];
                        }
                    }

                    if (v != u) {
                        istart = xadj[u];
                        iend = xadj[u + 1];
                        for (j = istart; j < iend; j++) {
                            k = cmap[adjncy[j]];
                            for (kk = k & mask; htable[kk] != -1 && cadjncy[htable[kk]] != k; kk = ((kk + 1) & mask));
                            if ((m = htable[kk]) == -1) {
                                cadjncy[nedges] = k;
                                cadjwgt[nedges] = adjwgt[j];
                                htable[kk] = nedges++;
                            }
                            else {
                                cadjwgt[m] += adjwgt[j];
                            }
                        }
                    }

                    /* reset the htable -- reverse order (LIFO) is critical to prevent cadjncy[-1]
                     * indexing due to a remove of an earlier entry */
                    for (j = nedges - 1; j >= 0; j--) {
                        k = cadjncy[j];
                        for (kk = k & mask; cadjncy[htable[kk]] != k; kk = ((kk + 1) & mask));
                        htable[kk] = -1;
                    }

                    /* remove the contracted vertex from the list */
                    cadjncy[0] = cadjncy[--nedges];
                    cadjwgt[0] = cadjwgt[nedges];
                }
                else {
                    nedges = 0;
                    istart = xadj[v];
                    iend = xadj[v + 1];
                    for (j = istart; j < iend; j++) {
                        k = cmap[adjncy[j]];
                        if ((m = dtable[k]) == -1) {
                            cadjncy[nedges] = k;
                            cadjwgt[nedges] = adjwgt[j];
                            dtable[k] = nedges++;
                        }
                        else {
                            cadjwgt[m] += adjwgt[j];
                        }
                    }

                    if (v != u) {
                        istart = xadj[u];
                        iend = xadj[u + 1];
                        for (j = istart; j < iend; j++) {
                            k = cmap[adjncy[j]];
                            if ((m = dtable[k]) == -1) {
                                cadjncy[nedges] = k;
                                cadjwgt[nedges] = adjwgt[j];
                                dtable[k] = nedges++;
                            }
                            else {
                                cadjwgt[m] += adjwgt[j];
                            }
                        }

                        /* Remove the contracted self-loop, when present */
                        if ((j = dtable[cnvtxs]) != -1) {
                            ASSERT(cadjncy[j] == cnvtxs);
                            cadjncy[j] = cadjncy[--nedges];
                            cadjwgt[j] = cadjwgt[nedges];
                            dtable[cnvtxs] = -1;
                        }
                    }

                    /* Zero out the dtable */
                    for (j = 0; j < nedges; j++)
                        dtable[cadjncy[j]] = -1;
                }


                /* Determine the median weight of the incident edges, which will be used
                   to keep an edge (u, v) iff wgt(u, v) >= min(medianewgts[u], medianewgts[v]) */
                if (dropedges) {
                    //ASSERTP(nedges < nkeys, ("%"PRIDX", %"PRIDX"\n", nkeys, nedges));
                    medianewgts[cnvtxs] = 8;  /* default for island nodes */
                    if (nedges > 0) {
                        for (j = 0; j < nedges; j++)
                            keys[j] = (cadjwgt[j] << 8) + noise[cnvtxs] + noise[cadjncy[j]];
                        isortd(nedges, keys);
                        medianewgts[cnvtxs] = keys[gk_min(nedges - 1, ((xadj[v + 1] - xadj[v] + xadj[u + 1] - xadj[u]) >> 1))];
                    }
                }

                cadjncy += nedges;
                cadjwgt += nedges;
                cnedges += nedges;
                cxadj[++cnvtxs] = cnedges;
            }


            /* compact the adjacency structure of the coarser graph to keep only +ve edges */
            if (dropedges) {
                droppedewgt = 0;

                cadjncy = cgraph->adjncy.data();
                cadjwgt = cgraph->adjwgt.data();

                cnedges = 0;
                for (u = 0; u < cnvtxs; u++) {
                    istart = cxadj[u];
                    iend = cxadj[u + 1];
                    for (j = istart; j < iend; j++) {
                        v = cadjncy[j];
                        // ASSERTP(medianewgts[u] >= 0, ("%"PRIDX" %"PRIDX"\n", u, medianewgts[u]));
                        // ASSERTP(medianewgts[v] >= 0, ("%"PRIDX" %"PRIDX" %"PRIDX"\n", v, medianewgts[v], cnvtxs));
                        if ((cadjwgt[j] << 8) + noise[u] + noise[v] >= gk_min(medianewgts[u], medianewgts[v])) {
                            cadjncy[cnedges] = cadjncy[j];
                            cadjwgt[cnedges++] = cadjwgt[j];
                        }
                        else
                            droppedewgt += cadjwgt[j];
                    }
                    cxadj[u] = cnedges;
                }
                SHIFTCSR(j, cnvtxs, cxadj);

                cgraph->droppedewgt = droppedewgt;
            }

            cgraph->nedges = cnedges;

            for (j = 0; j < ncon; j++) {
                cgraph->tvwgt[j] = isum(cgraph->nvtxs, cgraph->vwgt.data() + j, ncon);
                cgraph->invtvwgt[j] = 1.0f / (cgraph->tvwgt[j] > 0 ? cgraph->tvwgt[j] : 1);
            }

            ReAdjustMemory(ctrl, graph, cgraph);

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ContractTmr));

            WCOREPOP;
        }

        graph_t* CoarsenGraph(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, eqewgts, level = 0;

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->CoarsenTmr));

            /* determine if the weights on the edges are all the same */
            for (eqewgts = 1, i = 1; i < graph->nedges; i++) {
                if (graph->adjwgt[0] != graph->adjwgt[i]) {
                    eqewgts = 0;
                    break;
                }
            }

            /* set the maximum allowed coarsest vertex weight */
            for (i = 0; i < graph->ncon; i++)
                ctrl->maxvwgt[i] = (idx_t)(1.5f * graph->tvwgt[i] / ctrl->CoarsenTo);

            do {
                // IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));

                /* allocate memory for cmap, if it has not already been done due to
                   multiple cuts */
                if (graph->cmap.empty())
                    graph->cmap.resize(graph->nvtxs);

                /* determine which matching scheme you will use */
                switch (ctrl->ctype) {
                case METIS_CTYPE_RM:
                    Match_RM(ctrl, graph);
                    break;
                case METIS_CTYPE_SHEM:
                    if (eqewgts || graph->nedges == 0)
                        Match_RM(ctrl, graph);
                    else
                        Match_SHEM(ctrl, graph);
                    break;
                default:
                    gk_errexit("Unknown ctype: %d\n");
                }

                // graph_WriteToDisk(ctrl, graph);

                graph = graph->coarser;
                eqewgts = 0;
                level++;

                ASSERT(CheckGraph(graph, 0, 1));

            } while (graph->nvtxs > ctrl->CoarsenTo &&
                graph->nvtxs < COARSEN_FRACTION * graph->finer->nvtxs &&
                graph->nedges > graph->nvtxs / 2);

            // IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));
            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->CoarsenTmr));

            return graph;
        }


        idx_t Match_RM(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, pi, j, k, nvtxs, ncon, cnvtxs, maxidx,
                last_unmatched, avgdegree, bnum;
            idx_t* xadj, * vwgt, * adjncy, * maxvwgt;
            idx_t* cmap;
            std::vector<idx_t> match, degrees, perm, tperm;
            size_t nunmatched = 0;

            WCOREPUSH;

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            cmap = graph->cmap.data();

            maxvwgt = ctrl->maxvwgt.data();

            match.resize(nvtxs, UNMATCHED);
            perm.resize(nvtxs);
            tperm.resize(nvtxs);
            degrees.resize(nvtxs);

            /* Determine a "random" traversal order that is biased towards
               low-degree vertices */
            irandArrayPermute(nvtxs, tperm.data(), nvtxs / 8, 1);

            avgdegree = (idx_t)(4.0f * (xadj[nvtxs] / nvtxs));
            for (i = 0; i < nvtxs; i++) {
                bnum = (idx_t)sqrt(1 + xadj[i + 1] - xadj[i]);
                degrees[i] = (bnum > avgdegree ? avgdegree : bnum);
            }
            BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees.data(), tperm.data(), perm.data());


            /* Traverse the vertices and compute the matching */
            for (cnvtxs = 0, last_unmatched = 0, pi = 0; pi < nvtxs; pi++) {
                i = perm[pi];

                if (match[i] == UNMATCHED) {  /* Unmatched */
                    maxidx = i;

                    if ((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt + i * ncon, maxvwgt))) {
                        /* Deal with island vertices. Find a non-island and match it with.
                           The matching ignores ctrl->maxvwgt requirements */
                        if (xadj[i] == xadj[i + 1]) {
                            last_unmatched = gk_max(pi, last_unmatched) + 1;
                            for (; last_unmatched < nvtxs; last_unmatched++) {
                                j = perm[last_unmatched];
                                if (match[j] == UNMATCHED) {
                                    maxidx = j;
                                    break;
                                }
                            }
                        }
                        else {
                            /* Find a random matching, subject to maxvwgt constraints */
                            if (ncon == 1) {
                                /* single constraint version */
                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    k = adjncy[j];
                                    if (match[k] == UNMATCHED && vwgt[i] + vwgt[k] <= maxvwgt[0]) {
                                        maxidx = k;
                                        break;
                                    }
                                }

                                /* If it did not match, record for a 2-hop matching. */
                                if (maxidx == i && 2 * vwgt[i] < maxvwgt[0]) {
                                    nunmatched++;
                                    maxidx = UNMATCHED;
                                }
                            }
                            else {
                                /* multi-constraint version */
                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    k = adjncy[j];
                                    if (match[k] == UNMATCHED &&
                                        ivecaxpylez(ncon, 1, vwgt + i * ncon, vwgt + k * ncon, maxvwgt)) {
                                        maxidx = k;
                                        break;
                                    }
                                }

                                /* If it did not match, record for a 2-hop matching. */
                                if (maxidx == i && ivecaxpylez(ncon, 2, vwgt + i * ncon, vwgt + i * ncon, maxvwgt)) {
                                    nunmatched++;
                                    maxidx = UNMATCHED;
                                }
                            }
                        }
                    }

                    if (maxidx != UNMATCHED) {
                        cmap[i] = cmap[maxidx] = cnvtxs++;
                        match[i] = maxidx;
                        match[maxidx] = i;
                    }
                }
            }

            /* see if a 2-hop matching is required/allowed */
            if (!ctrl->no2hop && nunmatched > UNMATCHEDFOR2HOP * nvtxs)
                cnvtxs = Match_2Hop(ctrl, graph, perm.data(), match.data(), cnvtxs, nunmatched);


            /* match the final unmatched vertices with themselves and reorder the vertices
               of the coarse graph for memory-friendly contraction */
            for (cnvtxs = 0, i = 0; i < nvtxs; i++) {
                if (match[i] == UNMATCHED) {
                    match[i] = i;
                    cmap[i] = cnvtxs++;
                }
                else {
                    if (i <= match[i])
                        cmap[i] = cmap[match[i]] = cnvtxs++;
                }
            }

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

            CreateCoarseGraph(ctrl, graph, cnvtxs, match.data());

            WCOREPOP;

            return cnvtxs;
        }


        /**************************************************************************/
        /*! This function finds a matching using the HEM heuristic. The vertices
            are visited based on increasing degree to ensure that all vertices are
            given a chance to match with something.
         */
         /**************************************************************************/
        idx_t Match_SHEM(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, pi, j, k, nvtxs, ncon, cnvtxs, maxidx, maxwgt,
                last_unmatched, avgdegree, bnum;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * maxvwgt;
            idx_t* cmap;
            std::vector<idx_t> match, degrees, perm, tperm;
            size_t nunmatched = 0;

            WCOREPUSH;

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            cmap = graph->cmap.data();

            maxvwgt = ctrl->maxvwgt.data();

            match.resize(nvtxs, UNMATCHED);
            perm.resize(nvtxs);
            tperm.resize(nvtxs);
            degrees.resize(nvtxs);

            /* Determine a "random" traversal order that is biased towards low-degree vertices */
            irandArrayPermute(nvtxs, tperm.data(), nvtxs / 8, 1);

            avgdegree = (idx_t)(4.0f * (xadj[nvtxs] / nvtxs));
            for (i = 0; i < nvtxs; i++) {
                bnum = (idx_t)sqrt(1 + xadj[i + 1] - xadj[i]);
                degrees[i] = (bnum > avgdegree ? avgdegree : bnum);
            }
            BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees.data(), tperm.data(), perm.data());


            /* Traverse the vertices and compute the matching */
            for (cnvtxs = 0, last_unmatched = 0, pi = 0; pi < nvtxs; pi++) {
                i = perm[pi];

                if (match[i] == UNMATCHED) {  /* Unmatched */
                    maxidx = i;
                    maxwgt = -1;

                    if ((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt + i * ncon, maxvwgt))) {
                        /* Deal with island vertices. Find a non-island and match it with.
                           The matching ignores ctrl->maxvwgt requirements */
                        if (xadj[i] == xadj[i + 1]) {
                            last_unmatched = gk_max(pi, last_unmatched) + 1;
                            for (; last_unmatched < nvtxs; last_unmatched++) {
                                j = perm[last_unmatched];
                                if (match[j] == UNMATCHED) {
                                    maxidx = j;
                                    break;
                                }
                            }
                        }
                        else {
                            /* Find a heavy-edge matching, subject to maxvwgt constraints */
                            if (ncon == 1) {
                                /* single constraint version */
                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    k = adjncy[j];
                                    if (match[k] == UNMATCHED &&
                                        maxwgt < adjwgt[j] && vwgt[i] + vwgt[k] <= maxvwgt[0]) {
                                        maxidx = k;
                                        maxwgt = adjwgt[j];
                                    }
                                }

                                /* If it did not match, record for a 2-hop matching. */
                                if (maxidx == i && 2 * vwgt[i] < maxvwgt[0]) {
                                    nunmatched++;
                                    maxidx = UNMATCHED;
                                }
                            }
                            else {
                                /* multi-constraint version */
                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    k = adjncy[j];
                                    if (match[k] == UNMATCHED &&
                                        ivecaxpylez(ncon, 1, vwgt + i * ncon, vwgt + k * ncon, maxvwgt) &&
                                        (maxwgt < adjwgt[j] ||
                                            (maxwgt == adjwgt[j] &&
                                                BetterVBalance(ncon, graph->invtvwgt.data(), vwgt + i * ncon,
                                                    vwgt + maxidx * ncon, vwgt + k * ncon)))) {
                                        maxidx = k;
                                        maxwgt = adjwgt[j];
                                    }
                                }

                                /* If it did not match, record for a 2-hop matching. */
                                if (maxidx == i && ivecaxpylez(ncon, 2, vwgt + i * ncon, vwgt + i * ncon, maxvwgt)) {
                                    nunmatched++;
                                    maxidx = UNMATCHED;
                                }
                            }
                        }
                    }

                    if (maxidx != UNMATCHED) {
                        cmap[i] = cmap[maxidx] = cnvtxs++;
                        match[i] = maxidx;
                        match[maxidx] = i;
                    }
                }
            }

            /* see if a 2-hop matching is required/allowed */
            if (!ctrl->no2hop && nunmatched > UNMATCHEDFOR2HOP * nvtxs)
                cnvtxs = Match_2Hop(ctrl, graph, perm.data(), match.data(), cnvtxs, nunmatched);


            /* match the final unmatched vertices with themselves and reorder the vertices
               of the coarse graph for memory-friendly contraction */
            for (cnvtxs = 0, i = 0; i < nvtxs; i++) {
                if (match[i] == UNMATCHED) {
                    match[i] = i;
                    cmap[i] = cnvtxs++;
                }
                else {
                    if (i <= match[i])
                        cmap[i] = cmap[match[i]] = cnvtxs++;
                }
            }

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

            CreateCoarseGraph(ctrl, graph, cnvtxs, match.data());

            WCOREPOP;

            return cnvtxs;
        }


        /*************************************************************************/
        /*! This function matches the unmatched vertices using a 2-hop matching
            that involves vertices that are two hops away from each other. */
            /**************************************************************************/
        idx_t Match_2Hop(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match,
            idx_t cnvtxs, size_t nunmatched)
        {

            cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, 2);
            cnvtxs = Match_2HopAll(ctrl, graph, perm, match, cnvtxs, &nunmatched, 64);
            if (nunmatched > 1.5 * UNMATCHEDFOR2HOP * graph->nvtxs)
                cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, 3);
            if (nunmatched > 2.0 * UNMATCHEDFOR2HOP * graph->nvtxs)
                cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, graph->nvtxs);

            return cnvtxs;
        }


        /*************************************************************************/
        /*! This function matches the unmatched vertices whose degree is less than
            maxdegree using a 2-hop matching that involves vertices that are two
            hops away from each other.
            The requirement of the 2-hop matching is a simple non-empty overlap
            between the adjancency lists of the vertices. */
            /**************************************************************************/
        idx_t Match_2HopAny(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match,
            idx_t cnvtxs, size_t* r_nunmatched, size_t maxdegree)
        {
            idx_t i, pi, j, jj, nvtxs;
            idx_t* xadj, * adjncy;
            std::vector<idx_t> colptr, rowind;
            idx_t* cmap;
            size_t nunmatched;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            cmap = graph->cmap.data();

            nunmatched = *r_nunmatched;

            /* create the inverted index */
            WCOREPUSH;
            colptr.resize(nvtxs + 1, 0);
            for (i = 0; i < nvtxs; i++) {
                if (match[i] == UNMATCHED && xadj[i + 1] - xadj[i] < (idx_t)maxdegree) {
                    for (j = xadj[i]; j < xadj[i + 1]; j++)
                        colptr[adjncy[j]]++;
                }
            }
            MAKECSR(i, nvtxs, colptr);

            rowind.resize(colptr[nvtxs]);
            for (pi = 0; pi < nvtxs; pi++) {
                i = perm[pi];
                if (match[i] == UNMATCHED && xadj[i + 1] - xadj[i] < (idx_t)maxdegree) {
                    for (j = xadj[i]; j < xadj[i + 1]; j++)
                        rowind[colptr[adjncy[j]]++] = i;
                }
            }
            SHIFTCSR(i, nvtxs, colptr);

            /* compute matchings by going down the inverted index */
            for (pi = 0; pi < nvtxs; pi++) {
                i = perm[pi];
                if (colptr[i + 1] - colptr[i] < 2)
                    continue;

                for (jj = colptr[i + 1], j = colptr[i]; j < jj; j++) {
                    if (match[rowind[j]] == UNMATCHED) {
                        for (jj--; jj > j; jj--) {
                            if (match[rowind[jj]] == UNMATCHED) {
                                cmap[rowind[j]] = cmap[rowind[jj]] = cnvtxs++;
                                match[rowind[j]] = rowind[jj];
                                match[rowind[jj]] = rowind[j];
                                nunmatched -= 2;
                                break;
                            }
                        }
                    }
                }
            }
            WCOREPOP;

            *r_nunmatched = nunmatched;
            return cnvtxs;
        }


        /*************************************************************************/
        /*! This function matches the unmatched vertices whose degree is less than
            maxdegree using a 2-hop matching that involves vertices that are two
            hops away from each other.
            The requirement of the 2-hop matching is that of identical adjacency
            lists.
         */
         /**************************************************************************/
        idx_t Match_2HopAll(ctrl_t* ctrl, graph_t* graph, idx_t* perm, idx_t* match,
            idx_t cnvtxs, size_t* r_nunmatched, size_t maxdegree)
        {
            idx_t i, pi, pk, j, jj, k, nvtxs, mask, idegree;
            idx_t* xadj, * adjncy;
            idx_t* cmap;
            std::vector<idx_t> mark;
            std::vector<ikv_t> keys;
            size_t nunmatched, ncand;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            cmap = graph->cmap.data();

            nunmatched = *r_nunmatched;
            mask = (idx_t)(IDX_MAX / maxdegree);

            WCOREPUSH;

            /* collapse vertices with identical adjancency lists */
            keys.resize(nunmatched);
            for (ncand = 0, pi = 0; pi < nvtxs; pi++) {
                i = perm[pi];
                idegree = xadj[i + 1] - xadj[i];
                if (match[i] == UNMATCHED && idegree > 1 && idegree < (idx_t)maxdegree) {
                    for (k = 0, j = xadj[i]; j < xadj[i + 1]; j++)
                        k += adjncy[j] % mask;
                    keys[ncand].val = i;
                    keys[ncand].key = (int)((k % mask) * maxdegree + idegree);
                    ncand++;
                }
            }
            ikvsorti(ncand, keys.data());

            mark.resize(nvtxs, 0);
            for (pi = 0; pi < (idx_t)ncand; pi++) {
                i = keys[pi].val;
                if (match[i] != UNMATCHED)
                    continue;

                for (j = xadj[i]; j < xadj[i + 1]; j++)
                    mark[adjncy[j]] = i;

                for (pk = pi + 1; pk < (idx_t)ncand; pk++) {
                    k = keys[pk].val;
                    if (match[k] != UNMATCHED)
                        continue;

                    if (keys[pi].key != keys[pk].key)
                        break;
                    if (xadj[i + 1] - xadj[i] != xadj[k + 1] - xadj[k])
                        break;

                    for (jj = xadj[k]; jj < xadj[k + 1]; jj++) {
                        if (mark[adjncy[jj]] != i)
                            break;
                    }
                    if (jj == xadj[k + 1]) {
                        cmap[i] = cmap[k] = cnvtxs++;
                        match[i] = k;
                        match[k] = i;
                        nunmatched -= 2;
                        break;
                    }
                }
            }
            WCOREPOP;

            *r_nunmatched = nunmatched;
            return cnvtxs;
        }


        /*************************************************************************/
        /*! This function finds a matching by selecting an adjacent vertex based
            on the Jaccard coefficient of the adjaceny lists.
         */
         /**************************************************************************/
        idx_t Match_JC(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, pi, ii, iii, j, jj, jjj, k, nvtxs, ncon, cnvtxs, maxidx,
                last_unmatched, avgdegree, bnum;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * maxvwgt;
            idx_t* cmap, * vec, * marker;
            std::vector<idx_t> match, perm, tperm, degrees;
            idx_t mytwgt, xtwgt, ctwgt;
            real_t bscore, score;

            WCOREPUSH;

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            cmap = graph->cmap.data();

            maxvwgt = ctrl->maxvwgt.data();

            match.resize(nvtxs, UNMATCHED);
            perm.resize(nvtxs);
            tperm.resize(nvtxs);
            degrees.resize(nvtxs);

            irandArrayPermute(nvtxs, tperm.data(), nvtxs / 8, 1);

            avgdegree = (idx_t)(4.0 * (xadj[nvtxs] / nvtxs));
            for (i = 0; i < nvtxs; i++) {
                bnum = (idx_t)(sqrt(1 + xadj[i + 1] - xadj[i]));
                degrees[i] = (bnum > avgdegree ? avgdegree : bnum);
            }
            BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees.data(), tperm.data(), perm.data());

            /* point to the wspace vectors that are not needed any more */
            vec = tperm.data();
            marker = degrees.data();
            iset(nvtxs, -1, vec);
            iset(nvtxs, -1, marker);

            for (cnvtxs = 0, last_unmatched = 0, pi = 0; pi < nvtxs; pi++) {
                i = perm[pi];

                if (match[i] == UNMATCHED) {  /* Unmatched */
                    maxidx = i;

                    if ((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt + i * ncon, maxvwgt))) {
                        /* Deal with island vertices. Find a non-island and match it with.
                           The matching ignores ctrl->maxvwgt requirements */
                        if (xadj[i] == xadj[i + 1]) {
                            last_unmatched = gk_max(pi, last_unmatched) + 1;
                            for (; last_unmatched < nvtxs; last_unmatched++) {
                                j = perm[last_unmatched];
                                if (match[j] == UNMATCHED) {
                                    maxidx = j;
                                    break;
                                }
                            }
                        }
                        else {
                            if (ncon == 1) {
                                /* Find a max JC pair, subject to maxvwgt constraints */
                                if (xadj[i + 1] - xadj[i] < avgdegree) {
                                    marker[i] = i;
                                    bscore = 0.0;
                                    mytwgt = 0;
                                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                        mytwgt += 1;//adjwgt[j];
                                        vec[adjncy[j]] = 1;//adjwgt[j];
                                    }

                                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                        ii = adjncy[j];
                                        for (jj = xadj[ii]; jj < xadj[ii + 1]; jj++) {
                                            iii = adjncy[jj];

                                            if (marker[iii] == i || match[iii] != UNMATCHED || vwgt[i] + vwgt[iii] > maxvwgt[0])
                                                continue;

                                            ctwgt = xtwgt = 0;
                                            for (jjj = xadj[iii]; jjj < xadj[iii + 1]; jjj++) {
                                                xtwgt += 1;//adjwgt[jjj];
                                                if (vec[adjncy[jjj]] > 0)
                                                    ctwgt += 2;//vec[adjncy[jjj]] + adjwgt[jjj];
                                                else if (adjncy[jjj] == i)
                                                    ctwgt += 10 * adjwgt[jjj];
                                            }

                                            score = 1.0f * ctwgt / (mytwgt + xtwgt);
                                            if (score > bscore) {
                                                bscore = score;
                                                maxidx = iii;
                                            }
                                            marker[iii] = i;
                                        }
                                    }

                                    /* reset vec array */
                                    for (j = xadj[i]; j < xadj[i + 1]; j++)
                                        vec[adjncy[j]] = -1;
                                }
                            }
                            else {
                                /* multi-constraint version */
                                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                                    k = adjncy[j];
                                    if (match[k] == UNMATCHED &&
                                        ivecaxpylez(ncon, 1, vwgt + i * ncon, vwgt + k * ncon, maxvwgt)) {
                                        maxidx = k;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if (maxidx != UNMATCHED) {
                        cmap[i] = cmap[maxidx] = cnvtxs++;
                        match[i] = maxidx;
                        match[maxidx] = i;
                    }
                }
            }


            /* match the final unmatched vertices with themselves and reorder the vertices
               of the coarse graph for memory-friendly contraction */
            for (cnvtxs = 0, i = 0; i < nvtxs; i++) {
                if (match[i] == UNMATCHED) {
                    match[i] = i;
                    cmap[i] = cnvtxs++;
                }
                else {
                    if (i <= match[i])
                        cmap[i] = cmap[match[i]] = cnvtxs++;
                }
            }

            //IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

            CreateCoarseGraph(ctrl, graph, cnvtxs, match.data());

            WCOREPOP;

            return cnvtxs;
        }

        /*************************************************************************/
        /*! This function is the entry point of refinement */
        /*************************************************************************/
        void Refine2Way(ctrl_t* ctrl, graph_t* orggraph, graph_t* graph, real_t* tpwgts)
        {

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->UncoarsenTmr));

            /* Compute the parameters of the coarsest graph */
            Compute2WayPartitionParams(ctrl, graph);

            for (;;) {
                ASSERT(CheckBnd(graph));

                // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->RefTmr));

                Balance2Way(ctrl, graph, tpwgts);

                FM_2WayRefine(ctrl, graph, tpwgts, ctrl->niter);

                // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->RefTmr));

                if (graph == orggraph)
                    break;

                graph = graph->finer;
                // graph_ReadFromDisk(ctrl, graph);

                // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ProjectTmr));
                Project2WayPartition(ctrl, graph);
                // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ProjectTmr));
            }

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->UncoarsenTmr));
        }

        void Allocate2WayPartitionMemory(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t nvtxs, ncon;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;

            graph->pwgts.resize(2 * ncon);
            graph->where.resize(nvtxs);
            graph->bndptr.resize(nvtxs);
            graph->bndind.resize(nvtxs);
            graph->id.resize(nvtxs);
            graph->ed.resize(nvtxs);
        }

        void Compute2WayPartitionParams(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, j, nvtxs, ncon, nbnd, mincut, istart, iend, tid, ted, me;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * pwgts;
            idx_t* where, * bndptr, * bndind, * id, * ed;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();

            pwgts = iset(2 * ncon, 0, graph->pwgts.data());
            bndptr = iset(nvtxs, -1, graph->bndptr.data());
            bndind = graph->bndind.data();

            /* Compute pwgts */
            if (ncon == 1) {
                for (i = 0; i < nvtxs; i++) {
                    ASSERT(where[i] >= 0 && where[i] <= 1);
                    pwgts[where[i]] += vwgt[i];
                }
                ASSERT(pwgts[0] + pwgts[1] == graph->tvwgt[0]);
            }
            else {
                for (i = 0; i < nvtxs; i++) {
                    me = where[i];
                    for (j = 0; j < ncon; j++)
                        pwgts[me * ncon + j] += vwgt[i * ncon + j];
                }
            }


            /* Compute the required info for refinement  */
            for (nbnd = 0, mincut = 0, i = 0; i < nvtxs; i++) {
                istart = xadj[i];
                iend = xadj[i + 1];

                me = where[i];
                tid = ted = 0;

                for (j = istart; j < iend; j++) {
                    if (me == where[adjncy[j]])
                        tid += adjwgt[j];
                    else
                        ted += adjwgt[j];
                }
                id[i] = tid;
                ed[i] = ted;

                if (ted > 0 || istart == iend) {
                    BNDInsert(nbnd, bndind, bndptr, i);
                    mincut += ted;
                }
            }

            graph->mincut = mincut / 2;
            graph->nbnd = nbnd;

        }


        /*************************************************************************/
        /*! Projects a partition and computes the refinement params. */
        /*************************************************************************/
        void Project2WayPartition(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, j, istart, iend, nvtxs, nbnd, me, tid, ted;
            idx_t* xadj, * adjncy, * adjwgt;
            idx_t* cmap, * where, * bndptr, * bndind;
            idx_t* cwhere, * cbndptr;
            idx_t* id, * ed;
            graph_t* cgraph;
            int dropedges;

            Allocate2WayPartitionMemory(ctrl, graph);

            dropedges = ctrl->dropedges;

            cgraph = graph->coarser;
            cwhere = cgraph->where.data();
            cbndptr = cgraph->bndptr.data();

            nvtxs = graph->nvtxs;
            cmap = graph->cmap.data();
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();

            bndptr = iset(nvtxs, -1, graph->bndptr.data());
            bndind = graph->bndind.data();

            /* Project the partition and record which of these nodes came from the
               coarser boundary */
            for (i = 0; i < nvtxs; i++) {
                j = cmap[i];
                where[i] = cwhere[j];
                cmap[i] = (dropedges ? 0 : cbndptr[j]);
            }

            /* Compute the refinement information of the nodes */
            for (nbnd = 0, i = 0; i < nvtxs; i++) {
                istart = xadj[i];
                iend = xadj[i + 1];

                tid = ted = 0;
                if (cmap[i] == -1) { /* Interior node. Note that cmap[i] = cbndptr[cmap[i]] */
                    for (j = istart; j < iend; j++)
                        tid += adjwgt[j];
                }
                else { /* Potentially an interface node */
                    me = where[i];
                    for (j = istart; j < iend; j++) {
                        if (me == where[adjncy[j]])
                            tid += adjwgt[j];
                        else
                            ted += adjwgt[j];
                    }
                }
                id[i] = tid;
                ed[i] = ted;

                if (ted > 0 || istart == iend)
                    BNDInsert(nbnd, bndind, bndptr, i);
            }
            graph->mincut = (dropedges ? ComputeCut(graph, where) : cgraph->mincut);
            graph->nbnd = nbnd;

            /* copy pwgts */
            icopy(2 * graph->ncon, cgraph->pwgts.data(), graph->pwgts.data());

            delete graph->coarser;
            graph->coarser = NULL;
        }


        void FM_2WayRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter)
        {
            if (graph->ncon == 1)
                FM_2WayCutRefine(ctrl, graph, ntpwgts, niter);
            else
                FM_Mc2WayCutRefine(ctrl, graph, ntpwgts, niter);
        }


        /*************************************************************************/
        /*! This function performs a cut-focused FM refinement */
        /*************************************************************************/
        void FM_2WayCutRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter)
        {
            idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, limit, tmp;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * where, * id, * ed, * bndptr, * bndind, * pwgts;
            std::vector<idx_t> moved, swaps, perm;
            rpq_t* queues[2];
            idx_t higain, mincut, mindiff, origdiff, initcut, newcut, mincutorder, avgvwgt;
            idx_t tpwgts[2];

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();
            pwgts = graph->pwgts.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            moved.resize(nvtxs);
            swaps.resize(nvtxs);
            perm.resize(nvtxs);

            tpwgts[0] = (idx_t)(graph->tvwgt[0] * ntpwgts[0]);
            tpwgts[1] = graph->tvwgt[0] - tpwgts[0];

            limit = gk_min(gk_max((idx_t)(0.01 * nvtxs), 15), 100);
            avgvwgt = gk_min((pwgts[0] + pwgts[1]) / 20, 2 * (pwgts[0] + pwgts[1]) / nvtxs);

            queues[0] = rpqCreate(nvtxs);
            queues[1] = rpqCreate(nvtxs);

            //IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
            //    Print2WayRefineStats(ctrl, graph, ntpwgts, 0, -2));

            origdiff = iabs(tpwgts[0] - pwgts[0]);
            iset(nvtxs, -1, moved.data());
            for (pass = 0; pass < niter; pass++) { /* Do a number of passes */
                rpqReset(queues[0]);
                rpqReset(queues[1]);

                mincutorder = -1;
                newcut = mincut = initcut = graph->mincut;
                mindiff = iabs(tpwgts[0] - pwgts[0]);

                ASSERT(ComputeCut(graph, where) == graph->mincut);
                ASSERT(CheckBnd(graph));

                /* Insert boundary nodes in the priority queues */
                nbnd = graph->nbnd;
                irandArrayPermute(nbnd, perm.data(), nbnd, 1);
                for (ii = 0; ii < nbnd; ii++) {
                    i = perm[ii];
                    ASSERT(ed[bndind[i]] > 0 || id[bndind[i]] == 0);
                    ASSERT(bndptr[bndind[i]] != -1);
                    rpqInsert(queues[where[bndind[i]]], bndind[i], (float)ed[bndind[i]] - id[bndind[i]]);
                }

                for (nswaps = 0; nswaps < nvtxs; nswaps++) {
                    from = (tpwgts[0] - pwgts[0] < tpwgts[1] - pwgts[1] ? 0 : 1);
                    to = (from + 1) % 2;

                    if ((higain = rpqGetTop(queues[from])) == -1)
                        break;
                    ASSERT(bndptr[higain] != -1);

                    newcut -= (ed[higain] - id[higain]);
                    INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

                    if ((newcut < mincut && iabs(tpwgts[0] - pwgts[0]) <= origdiff + avgvwgt) ||
                        (newcut == mincut && iabs(tpwgts[0] - pwgts[0]) < mindiff)) {
                        mincut = newcut;
                        mindiff = iabs(tpwgts[0] - pwgts[0]);
                        mincutorder = nswaps;
                    }
                    else if (nswaps - mincutorder > limit) { /* We hit the limit, undo last move */
                        newcut += (ed[higain] - id[higain]);
                        INC_DEC(pwgts[from], pwgts[to], vwgt[higain]);
                        break;
                    }

                    where[higain] = to;
                    moved[higain] = nswaps;
                    swaps[nswaps] = higain;

                    /**************************************************************
                    * Update the id[i]/ed[i] values of the affected nodes
                    ***************************************************************/
                    SWAP(id[higain], ed[higain], tmp);
                    if (ed[higain] == 0 && xadj[higain] < xadj[higain + 1])
                        BNDDelete(nbnd, bndind, bndptr, higain);

                    for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                        k = adjncy[j];

                        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                        INC_DEC(id[k], ed[k], kwgt);

                        /* Update its boundary information and queue position */
                        if (bndptr[k] != -1) { /* If k was a boundary vertex */
                            if (ed[k] == 0) { /* Not a boundary vertex any more */
                                BNDDelete(nbnd, bndind, bndptr, k);
                                if (moved[k] == -1)  /* Remove it if in the queues */
                                    rpqDelete(queues[where[k]], k);
                            }
                            else { /* If it has not been moved, update its position in the queue */
                                if (moved[k] == -1)
                                    rpqUpdate(queues[where[k]], k, (float)ed[k] - id[k]);
                            }
                        }
                        else {
                            if (ed[k] > 0) {  /* It will now become a boundary vertex */
                                BNDInsert(nbnd, bndind, bndptr, k);
                                if (moved[k] == -1)
                                    rpqInsert(queues[where[k]], k, (float)ed[k] - id[k]);
                            }
                        }
                    }

                }


                /****************************************************************
                * Roll back computations
                *****************************************************************/
                for (i = 0; i < nswaps; i++)
                    moved[swaps[i]] = -1;  /* reset moved array */
                for (nswaps--; nswaps > mincutorder; nswaps--) {
                    higain = swaps[nswaps];

                    to = where[higain] = (where[higain] + 1) % 2;
                    SWAP(id[higain], ed[higain], tmp);
                    if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
                        BNDDelete(nbnd, bndind, bndptr, higain);
                    else if (ed[higain] > 0 && bndptr[higain] == -1)
                        BNDInsert(nbnd, bndind, bndptr, higain);

                    INC_DEC(pwgts[to], pwgts[(to + 1) % 2], vwgt[higain]);
                    for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                        k = adjncy[j];

                        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                        INC_DEC(id[k], ed[k], kwgt);

                        if (bndptr[k] != -1 && ed[k] == 0)
                            BNDDelete(nbnd, bndind, bndptr, k);
                        if (bndptr[k] == -1 && ed[k] > 0)
                            BNDInsert(nbnd, bndind, bndptr, k);
                    }
                }

                graph->mincut = mincut;
                graph->nbnd = nbnd;

                //IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
                //    Print2WayRefineStats(ctrl, graph, ntpwgts, 0, mincutorder));

                if (mincutorder <= 0 || mincut == initcut)
                    break;
            }

            rpqDestroy(queues[0]);
            rpqDestroy(queues[1]);

            WCOREPOP;
        }


        /*************************************************************************/
        /*! This function performs a cut-focused multi-constraint FM refinement */
        /*************************************************************************/
        void FM_Mc2WayCutRefine(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts, idx_t niter)
        {
            idx_t i, ii, j, k, kwgt, nvtxs, ncon, nbnd, nswaps, from, to, pass,
                limit, tmp, cnum;
            idx_t* xadj, * adjncy, * vwgt, * adjwgt, * pwgts, * where, * id, * ed,
                * bndptr, * bndind;
            std::vector<idx_t> moved, swaps, perm, qnum;
            idx_t higain, mincut, initcut, newcut, mincutorder;
            real_t* invtvwgt;
            std::vector<real_t> ubfactors, minbalv, newbalv;
            real_t minbal, newbal, rgain, ffactor;
            std::vector<rpq_t*> queues;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            invtvwgt = graph->invtvwgt.data();
            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();
            pwgts = graph->pwgts.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            moved.resize(nvtxs);
            swaps.resize(nvtxs);
            perm.resize(nvtxs);
            qnum.resize(nvtxs);
            ubfactors.resize(ncon);
            newbalv.resize(ncon);
            minbalv.resize(ncon);

            limit = gk_min(gk_max((idx_t)(0.01 * nvtxs), 25), 150);


            /* Determine a fudge factor to allow the refinement routines to get out
               of tight balancing constraints. */
            ffactor = 0.5f / gk_max(20, nvtxs);

            /* Initialize the queues */
            queues.resize(2 * ncon);
            for (i = 0; i < 2 * ncon; i++)
                queues[i] = rpqCreate(nvtxs);
            for (i = 0; i < nvtxs; i++)
                qnum[i] = iargmax_nrm(ncon, vwgt + i * ncon, invtvwgt);

            /* Determine the unbalance tolerance for each constraint. The tolerance is
               equal to the maximum of the original load imbalance and the user-supplied
               allowed tolerance. The rationale behind this approach is to allow the
               refinement routine to improve the cut, without having to worry about fixing
               load imbalance problems. The load imbalance is addressed by the balancing
               routines. */
            ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm.data(), ctrl->ubfactors.data(), ubfactors.data());
            for (i = 0; i < ncon; i++)
                ubfactors[i] = (ubfactors[i] > 0 ? ctrl->ubfactors[i] + ubfactors[i] : ctrl->ubfactors[i]);


            //IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
           //     Print2WayRefineStats(ctrl, graph, ntpwgts, origbal, -2));

            iset(nvtxs, -1, moved.data());
            for (pass = 0; pass < niter; pass++) { /* Do a number of passes */
                for (i = 0; i < 2 * ncon; i++)
                    rpqReset(queues[i]);

                mincutorder = -1;
                newcut = mincut = initcut = graph->mincut;

                minbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm.data(), ubfactors.data(), minbalv.data());

                ASSERT(ComputeCut(graph, where) == graph->mincut);
                ASSERT(CheckBnd(graph));

                /* Insert boundary nodes in the priority queues */
                nbnd = graph->nbnd;
                irandArrayPermute(nbnd, perm.data(), nbnd / 5, 1);
                for (ii = 0; ii < nbnd; ii++) {
                    i = bndind[perm[ii]];
                    ASSERT(ed[i] > 0 || id[i] == 0);
                    ASSERT(bndptr[i] != -1);
                    //rgain = 1.0*(ed[i]-id[i])/sqrt(vwgt[i*ncon+qnum[i]]+1);
                    //rgain = (ed[i]-id[i] > 0 ? 1.0*(ed[i]-id[i])/sqrt(vwgt[i*ncon+qnum[i]]+1) : ed[i]-id[i]);
                    rgain = (float)(ed[i] - id[i]);
                    rpqInsert(queues[2 * qnum[i] + where[i]], i, rgain);
                }

                for (nswaps = 0; nswaps < nvtxs; nswaps++) {
                    SelectQueue(graph, ctrl->pijbm.data(), ubfactors.data(), queues.data(), &from, &cnum);

                    to = (from + 1) % 2;

                    if (from == -1 || (higain = rpqGetTop(queues[2 * cnum + from])) == -1)
                        break;
                    ASSERT(bndptr[higain] != -1);

                    newcut -= (ed[higain] - id[higain]);

                    iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
                    iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
                    newbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm.data(), ubfactors.data(), newbalv.data());

                    if ((newcut < mincut && newbal <= ffactor) ||
                        (newcut == mincut && (newbal < minbal ||
                            (newbal == minbal && BetterBalance2Way(ncon, minbalv.data(), newbalv.data()))))) {
                        mincut = newcut;
                        minbal = newbal;
                        mincutorder = nswaps;
                        rcopy(ncon, newbalv.data(), minbalv.data());
                    }
                    else if (nswaps - mincutorder > limit) { /* We hit the limit, undo last move */
                        newcut += (ed[higain] - id[higain]);
                        iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
                        iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
                        break;
                    }

                    where[higain] = to;
                    moved[higain] = nswaps;
                    swaps[nswaps] = higain;

                    /**************************************************************
                    * Update the id[i]/ed[i] values of the affected nodes
                    ***************************************************************/
                    SWAP(id[higain], ed[higain], tmp);
                    if (ed[higain] == 0 && xadj[higain] < xadj[higain + 1])
                        BNDDelete(nbnd, bndind, bndptr, higain);

                    for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                        k = adjncy[j];

                        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                        INC_DEC(id[k], ed[k], kwgt);

                        /* Update its boundary information and queue position */
                        if (bndptr[k] != -1) { /* If k was a boundary vertex */
                            if (ed[k] == 0) { /* Not a boundary vertex any more */
                                BNDDelete(nbnd, bndind, bndptr, k);
                                if (moved[k] == -1)  /* Remove it if in the queues */
                                    rpqDelete(queues[2 * qnum[k] + where[k]], k);
                            }
                            else { /* If it has not been moved, update its position in the queue */
                                if (moved[k] == -1) {
                                    //rgain = 1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1);
                                    //rgain = (ed[k]-id[k] > 0 ? 
                                    //              1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1) : ed[k]-id[k]);
                                    rgain = (float)(ed[k] - id[k]);
                                    rpqUpdate(queues[2 * qnum[k] + where[k]], k, rgain);
                                }
                            }
                        }
                        else {
                            if (ed[k] > 0) {  /* It will now become a boundary vertex */
                                BNDInsert(nbnd, bndind, bndptr, k);
                                if (moved[k] == -1) {
                                    //rgain = 1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1);
                                    //rgain = (ed[k]-id[k] > 0 ? 
                                    //              1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1) : ed[k]-id[k]);
                                    rgain = (float)(ed[k] - id[k]);
                                    rpqInsert(queues[2 * qnum[k] + where[k]], k, rgain);
                                }
                            }
                        }
                    }

                }


                /****************************************************************
                * Roll back computations
                *****************************************************************/
                for (i = 0; i < nswaps; i++)
                    moved[swaps[i]] = -1;  /* reset moved array */
                for (nswaps--; nswaps > mincutorder; nswaps--) {
                    higain = swaps[nswaps];

                    to = where[higain] = (where[higain] + 1) % 2;
                    SWAP(id[higain], ed[higain], tmp);
                    if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
                        BNDDelete(nbnd, bndind, bndptr, higain);
                    else if (ed[higain] > 0 && bndptr[higain] == -1)
                        BNDInsert(nbnd, bndind, bndptr, higain);

                    iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
                    iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + ((to + 1) % 2) * ncon, 1);
                    for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                        k = adjncy[j];

                        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                        INC_DEC(id[k], ed[k], kwgt);

                        if (bndptr[k] != -1 && ed[k] == 0)
                            BNDDelete(nbnd, bndind, bndptr, k);
                        if (bndptr[k] == -1 && ed[k] > 0)
                            BNDInsert(nbnd, bndind, bndptr, k);
                    }
                }

                graph->mincut = mincut;
                graph->nbnd = nbnd;

                //   IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
                //       Print2WayRefineStats(ctrl, graph, ntpwgts, minbal, mincutorder));

                if (mincutorder <= 0 || mincut == initcut)
                    break;
            }

            for (i = 0; i < 2 * ncon; i++)
                rpqDestroy(queues[i]);

            WCOREPOP;
        }

        void Balance2Way(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
        {
            if (ComputeLoadImbalanceDiff(graph, 2, ctrl->pijbm.data(), ctrl->ubfactors.data()) <= 0)
                return;

            if (graph->ncon == 1) {
                /* return right away if the balance is OK */
                if (rabs(ntpwgts[0] * graph->tvwgt[0] - graph->pwgts[0]) < 3 * graph->tvwgt[0] / graph->nvtxs)
                    return;

                if (graph->nbnd > 0)
                    Bnd2WayBalance(ctrl, graph, ntpwgts);
                else
                    General2WayBalance(ctrl, graph, ntpwgts);
            }
            else {
                McGeneral2WayBalance(ctrl, graph, ntpwgts);
            }
        }


        /*************************************************************************
        * This function balances two partitions by moving boundary nodes
        * from the domain that is overweight to the one that is underweight.
        **************************************************************************/
        void Bnd2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
        {
            idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, tmp;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * where, * id, * ed, * bndptr, * bndind, * pwgts;
            std::vector<idx_t> moved, perm;
            rpq_t* queue;
            idx_t higain, mincut, mindiff;
            idx_t tpwgts[2];

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();
            pwgts = graph->pwgts.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            moved.resize(nvtxs);
            perm.resize(nvtxs);

            /* Determine from which domain you will be moving data */
            tpwgts[0] = (idx_t)(graph->tvwgt[0] * ntpwgts[0]);
            tpwgts[1] = graph->tvwgt[0] - tpwgts[0];
            mindiff = iabs(tpwgts[0] - pwgts[0]);
            from = (pwgts[0] < tpwgts[0] ? 1 : 0);
            to = (from + 1) % 2;
            queue = rpqCreate(nvtxs);

            iset(nvtxs, -1, moved.data());

            ASSERT(ComputeCut(graph, where) == graph->mincut);
            ASSERT(CheckBnd(graph));

            /* Insert the boundary nodes of the proper partition whose size is OK in the priority queue */
            nbnd = graph->nbnd;
            irandArrayPermute(nbnd, perm.data(), nbnd / 5, 1);
            for (ii = 0; ii < nbnd; ii++) {
                i = perm[ii];
                ASSERT(ed[bndind[i]] > 0 || id[bndind[i]] == 0);
                ASSERT(bndptr[bndind[i]] != -1);
                if (where[bndind[i]] == from && vwgt[bndind[i]] <= mindiff)
                    rpqInsert(queue, bndind[i], (float)ed[bndind[i]] - id[bndind[i]]);
            }

            mincut = graph->mincut;
            for (nswaps = 0; nswaps < nvtxs; nswaps++) {
                if ((higain = rpqGetTop(queue)) == -1)
                    break;
                ASSERT(bndptr[higain] != -1);

                if (pwgts[to] + vwgt[higain] > tpwgts[to])
                    break;

                mincut -= (ed[higain] - id[higain]);
                INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

                where[higain] = to;
                moved[higain] = nswaps;

                /**************************************************************
                * Update the id[i]/ed[i] values of the affected nodes
                ***************************************************************/
                SWAP(id[higain], ed[higain], tmp);
                if (ed[higain] == 0 && xadj[higain] < xadj[higain + 1])
                    BNDDelete(nbnd, bndind, bndptr, higain);

                for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                    k = adjncy[j];
                    kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                    INC_DEC(id[k], ed[k], kwgt);

                    /* Update its boundary information and queue position */
                    if (bndptr[k] != -1) { /* If k was a boundary vertex */
                        if (ed[k] == 0) { /* Not a boundary vertex any more */
                            BNDDelete(nbnd, bndind, bndptr, k);
                            if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)  /* Remove it if in the queues */
                                rpqDelete(queue, k);
                        }
                        else { /* If it has not been moved, update its position in the queue */
                            if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
                                rpqUpdate(queue, k, (float)ed[k] - id[k]);
                        }
                    }
                    else {
                        if (ed[k] > 0) {  /* It will now become a boundary vertex */
                            BNDInsert(nbnd, bndind, bndptr, k);
                            if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
                                rpqInsert(queue, k, (float)ed[k] - id[k]);
                        }
                    }
                }
            }

            graph->mincut = mincut;
            graph->nbnd = nbnd;

            rpqDestroy(queue);

            WCOREPOP;
        }


        /*************************************************************************
        * This function balances two partitions by moving the highest gain
        * (including negative gain) vertices to the other domain.
        * It is used only when the unbalance is due to non contiguous
        * subdomains. That is, the are no boundary vertices.
        * It moves vertices from the domain that is overweight to the one that
        * is underweight.
        **************************************************************************/
        void General2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
        {
            idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, tmp;
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * where, * id, * ed, * bndptr, * bndind, * pwgts;
            std::vector<idx_t> moved, perm;
            rpq_t* queue;
            idx_t higain, mincut, mindiff;
            idx_t tpwgts[2];

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();
            pwgts = graph->pwgts.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            moved.resize(nvtxs);
            perm.resize(nvtxs);

            /* Determine from which domain you will be moving data */
            tpwgts[0] = (idx_t)(graph->tvwgt[0] * ntpwgts[0]);
            tpwgts[1] = graph->tvwgt[0] - tpwgts[0];
            mindiff = iabs(tpwgts[0] - pwgts[0]);
            from = (pwgts[0] < tpwgts[0] ? 1 : 0);
            to = (from + 1) % 2;

            queue = rpqCreate(nvtxs);

            iset(nvtxs, -1, moved.data());

            ASSERT(ComputeCut(graph, where) == graph->mincut);
            ASSERT(CheckBnd(graph));

            /* Insert the nodes of the proper partition whose size is OK in the priority queue */
            irandArrayPermute(nvtxs, perm.data(), nvtxs / 5, 1);
            for (ii = 0; ii < nvtxs; ii++) {
                i = perm[ii];
                if (where[i] == from && vwgt[i] <= mindiff)
                    rpqInsert(queue, i, (float)ed[i] - id[i]);
            }

            mincut = graph->mincut;
            nbnd = graph->nbnd;
            for (nswaps = 0; nswaps < nvtxs; nswaps++) {
                if ((higain = rpqGetTop(queue)) == -1)
                    break;

                if (pwgts[to] + vwgt[higain] > tpwgts[to])
                    break;

                mincut -= (ed[higain] - id[higain]);
                INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

                where[higain] = to;
                moved[higain] = nswaps;

                /**************************************************************
                * Update the id[i]/ed[i] values of the affected nodes
                ***************************************************************/
                SWAP(id[higain], ed[higain], tmp);
                if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
                    BNDDelete(nbnd, bndind, bndptr, higain);
                if (ed[higain] > 0 && bndptr[higain] == -1)
                    BNDInsert(nbnd, bndind, bndptr, higain);

                for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                    k = adjncy[j];

                    kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                    INC_DEC(id[k], ed[k], kwgt);

                    /* Update the queue position */
                    if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
                        rpqUpdate(queue, k, (float)ed[k] - id[k]);

                    /* Update its boundary information */
                    if (ed[k] == 0 && bndptr[k] != -1)
                        BNDDelete(nbnd, bndind, bndptr, k);
                    else if (ed[k] > 0 && bndptr[k] == -1)
                        BNDInsert(nbnd, bndind, bndptr, k);
                }
            }

            graph->mincut = mincut;
            graph->nbnd = nbnd;

            rpqDestroy(queue);

            WCOREPOP;
        }


        /*************************************************************************
        * This function performs an edge-based FM refinement
        **************************************************************************/
        void McGeneral2WayBalance(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts)
        {
            idx_t i, ii, j, k, kwgt, nvtxs, ncon, nbnd, nswaps, from, to, limit, tmp, cnum;
            idx_t* xadj, * adjncy, * vwgt, * adjwgt, * where, * pwgts, * id, * ed, * bndptr, * bndind;
            std::vector<idx_t> moved, swaps, perm, qnum, qsizes;
            std::vector<real_t> minbalv, newbalv;
            idx_t higain, mincut, newcut, mincutorder;
            real_t* invtvwgt, minbal, newbal;
            std::vector<rpq_t*> queues;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            invtvwgt = graph->invtvwgt.data();
            where = graph->where.data();
            id = graph->id.data();
            ed = graph->ed.data();
            pwgts = graph->pwgts.data();
            bndptr = graph->bndptr.data();
            bndind = graph->bndind.data();

            moved.resize(nvtxs);
            swaps.resize(nvtxs);
            perm.resize(nvtxs);
            qnum.resize(nvtxs);
            newbalv.resize(ncon);
            minbalv.resize(ncon);
            qsizes.resize(2 * ncon);

            limit = gk_min(gk_max((idx_t)(0.01 * nvtxs), 15), 100);

            /* Initialize the queues */
            queues.resize(2 * ncon);
            for (i = 0; i < 2 * ncon; i++) {
                queues[i] = rpqCreate(nvtxs);
                qsizes[i] = 0;
            }

            for (i = 0; i < nvtxs; i++) {
                qnum[i] = iargmax_nrm(ncon, vwgt + i * ncon, invtvwgt);
                qsizes[2 * qnum[i] + where[i]]++;
            }


            /* for the empty queues, move into them vertices from other queues */
            for (from = 0; from < 2; from++) {
                for (j = 0; j < ncon; j++) {
                    if (qsizes[2 * j + from] == 0) {
                        for (i = 0; i < nvtxs; i++) {
                            if (where[i] != from)
                                continue;

                            k = iargmax2_nrm(ncon, vwgt + i * ncon, invtvwgt);
                            if (k == j &&
                                qsizes[2 * qnum[i] + from] > qsizes[2 * j + from] &&
                                vwgt[i * ncon + qnum[i]] * invtvwgt[qnum[i]] < 1.3 * vwgt[i * ncon + j] * invtvwgt[j]) {
                                qsizes[2 * qnum[i] + from]--;
                                qsizes[2 * j + from]++;
                                qnum[i] = j;
                            }
                        }
                    }
                }
            }


            minbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm.data(), ctrl->ubfactors.data(), minbalv.data());
            ASSERT(minbal > 0.0);

            newcut = mincut = graph->mincut;
            mincutorder = -1;

            iset(nvtxs, -1, moved.data());

            ASSERT(ComputeCut(graph, where) == graph->mincut);
            ASSERT(CheckBnd(graph));

            /* Insert all nodes in the priority queues */
            nbnd = graph->nbnd;
            irandArrayPermute(nvtxs, perm.data(), nvtxs / 10, 1);
            for (ii = 0; ii < nvtxs; ii++) {
                i = perm[ii];
                rpqInsert(queues[2 * qnum[i] + where[i]], i, (float)ed[i] - id[i]);
            }

            for (nswaps = 0; nswaps < nvtxs; nswaps++) {
                if (minbal <= 0.0)
                    break;

                SelectQueue(graph, ctrl->pijbm.data(), ctrl->ubfactors.data(), queues.data(), &from, &cnum);
                to = (from + 1) % 2;

                if (from == -1 || (higain = rpqGetTop(queues[2 * cnum + from])) == -1)
                    break;

                newcut -= (ed[higain] - id[higain]);

                iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
                iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
                newbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm.data(), ctrl->ubfactors.data(), newbalv.data());

                if (newbal < minbal || (newbal == minbal &&
                    (newcut < mincut ||
                        (newcut == mincut && BetterBalance2Way(ncon, minbalv.data(), newbalv.data()))))) {
                    mincut = newcut;
                    minbal = newbal;
                    mincutorder = nswaps;
                    rcopy(ncon, newbalv.data(), minbalv.data());
                }
                else if (nswaps - mincutorder > limit) { /* We hit the limit, undo last move */
                    newcut += (ed[higain] - id[higain]);
                    iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + from * ncon, 1);
                    iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
                    break;
                }

                where[higain] = to;
                moved[higain] = nswaps;
                swaps[nswaps] = higain;

                /**************************************************************
                * Update the id[i]/ed[i] values of the affected nodes
                ***************************************************************/
                SWAP(id[higain], ed[higain], tmp);
                if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
                    BNDDelete(nbnd, bndind, bndptr, higain);
                if (ed[higain] > 0 && bndptr[higain] == -1)
                    BNDInsert(nbnd, bndind, bndptr, higain);

                for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                    k = adjncy[j];

                    kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                    INC_DEC(id[k], ed[k], kwgt);

                    /* Update the queue position */
                    if (moved[k] == -1)
                        rpqUpdate(queues[2 * qnum[k] + where[k]], k, (float)ed[k] - id[k]);

                    /* Update its boundary information */
                    if (ed[k] == 0 && bndptr[k] != -1)
                        BNDDelete(nbnd, bndind, bndptr, k);
                    else if (ed[k] > 0 && bndptr[k] == -1)
                        BNDInsert(nbnd, bndind, bndptr, k);
                }
            }



            /****************************************************************
            * Roll back computations
            *****************************************************************/
            for (nswaps--; nswaps > mincutorder; nswaps--) {
                higain = swaps[nswaps];

                to = where[higain] = (where[higain] + 1) % 2;
                SWAP(id[higain], ed[higain], tmp);
                if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain + 1])
                    BNDDelete(nbnd, bndind, bndptr, higain);
                else if (ed[higain] > 0 && bndptr[higain] == -1)
                    BNDInsert(nbnd, bndind, bndptr, higain);

                iaxpy(ncon, 1, vwgt + higain * ncon, 1, pwgts + to * ncon, 1);
                iaxpy(ncon, -1, vwgt + higain * ncon, 1, pwgts + ((to + 1) % 2) * ncon, 1);
                for (j = xadj[higain]; j < xadj[higain + 1]; j++) {
                    k = adjncy[j];

                    kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
                    INC_DEC(id[k], ed[k], kwgt);

                    if (bndptr[k] != -1 && ed[k] == 0)
                        BNDDelete(nbnd, bndind, bndptr, k);
                    if (bndptr[k] == -1 && ed[k] > 0)
                        BNDInsert(nbnd, bndind, bndptr, k);
                }
            }

            graph->mincut = mincut;
            graph->nbnd = nbnd;


            for (i = 0; i < 2 * ncon; i++)
                rpqDestroy(queues[i]);

            WCOREPOP;
        }


        void RandomBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts,
            idx_t niparts)
        {
            idx_t i, ii, nvtxs, pwgts[2], zeromaxpwgt,
                bestcut = 0, inbfs;
            idx_t* vwgt, * where;
            std::vector<idx_t> perm, bestwhere;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            vwgt = graph->vwgt.data();

            Allocate2WayPartitionMemory(ctrl, graph);
            where = graph->where.data();

            bestwhere.resize(nvtxs);
            perm.resize(nvtxs);

            zeromaxpwgt = (idx_t)(ctrl->ubfactors[0] * graph->tvwgt[0] * ntpwgts[0]);

            for (inbfs = 0; inbfs < niparts; inbfs++) {
                iset(nvtxs, 1, where);

                if (inbfs > 0) {
                    irandArrayPermute(nvtxs, perm.data(), nvtxs / 2, 1);
                    pwgts[1] = graph->tvwgt[0];
                    pwgts[0] = 0;

                    for (ii = 0; ii < nvtxs; ii++) {
                        i = perm[ii];
                        if (pwgts[0] + vwgt[i] < zeromaxpwgt) {
                            where[i] = 0;
                            pwgts[0] += vwgt[i];
                            pwgts[1] -= vwgt[i];
                            if (pwgts[0] > zeromaxpwgt)
                                break;
                        }
                    }
                }

                /* Do some partition refinement  */
                Compute2WayPartitionParams(ctrl, graph);

                Balance2Way(ctrl, graph, ntpwgts);

                FM_2WayRefine(ctrl, graph, ntpwgts, 4);

                if (inbfs == 0 || bestcut > graph->mincut) {
                    bestcut = graph->mincut;
                    icopy(nvtxs, where, bestwhere.data());
                    if (bestcut == 0)
                        break;
                }
            }

            graph->mincut = bestcut;
            icopy(nvtxs, bestwhere.data(), where);

            WCOREPOP;
        }

        /*************************************************************************/
        /*! This function takes a graph and produces a bisection by using a region
            growing algorithm. The resulting bisection is refined using FM.
            The resulting partition is returned in graph->where.
        */
        /*************************************************************************/
        void GrowBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts,
            idx_t niparts)
        {
            idx_t i, j, k, nvtxs, drain, nleft, first, last,
                pwgts[2], oneminpwgt, onemaxpwgt,
                bestcut = 0, inbfs;
            idx_t* xadj, * vwgt, * adjncy, * where;
            std::vector<idx_t> queue, touched, bestwhere;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            // idx_t *adjwgt = graph->adjwgt.data();

            Allocate2WayPartitionMemory(ctrl, graph);
            where = graph->where.data();

            bestwhere.resize(nvtxs);
            queue.resize(nvtxs);
            touched.resize(nvtxs);

            onemaxpwgt = (idx_t)(ctrl->ubfactors[0] * graph->tvwgt[0] * ntpwgts[1]);
            oneminpwgt = (idx_t)((1.0 / ctrl->ubfactors[0]) * graph->tvwgt[0] * ntpwgts[1]);

            for (inbfs = 0; inbfs < niparts; inbfs++) {
                iset(nvtxs, 1, where);

                iset(nvtxs, 0, touched.data());

                pwgts[1] = graph->tvwgt[0];
                pwgts[0] = 0;

                queue[0] = irandInRange(nvtxs);
                touched[queue[0]] = 1;
                first = 0;
                last = 1;
                nleft = nvtxs - 1;
                drain = 0;

                /* Start the BFS from queue to get a partition */
                for (;;) {
                    if (first == last) { /* Empty. Disconnected graph! */
                        if (nleft == 0 || drain)
                            break;

                        k = irandInRange(nleft);
                        for (i = 0; i < nvtxs; i++) {
                            if (touched[i] == 0) {
                                if (k == 0)
                                    break;
                                else
                                    k--;
                            }
                        }

                        queue[0] = i;
                        touched[i] = 1;
                        first = 0;
                        last = 1;
                        nleft--;
                    }

                    i = queue[first++];
                    if (pwgts[0] > 0 && pwgts[1] - vwgt[i] < oneminpwgt) {
                        drain = 1;
                        continue;
                    }

                    where[i] = 0;
                    INC_DEC(pwgts[0], pwgts[1], vwgt[i]);
                    if (pwgts[1] <= onemaxpwgt)
                        break;

                    drain = 0;
                    for (j = xadj[i]; j < xadj[i + 1]; j++) {
                        k = adjncy[j];
                        if (touched[k] == 0) {
                            queue[last++] = k;
                            touched[k] = 1;
                            nleft--;
                        }
                    }
                }

                /* Check to see if we hit any bad limiting cases */
                if (pwgts[1] == 0)
                    where[irandInRange(nvtxs)] = 1;
                if (pwgts[0] == 0)
                    where[irandInRange(nvtxs)] = 0;

                /*************************************************************
                * Do some partition refinement
                **************************************************************/
                Compute2WayPartitionParams(ctrl, graph);

                Balance2Way(ctrl, graph, ntpwgts);

                FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

                if (inbfs == 0 || bestcut > graph->mincut) {
                    bestcut = graph->mincut;
                    icopy(nvtxs, where, bestwhere.data());
                    if (bestcut == 0)
                        break;
                }
            }

            graph->mincut = bestcut;
            icopy(nvtxs, bestwhere.data(), where);

            WCOREPOP;
        }



        /*************************************************************************/
        /*! This function takes a multi-constraint graph and computes a bisection
            by randomly assigning the vertices and then refining it. The resulting
            partition is returned in graph->where.
        */
        /**************************************************************************/
        void McRandomBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts,
            idx_t niparts)
        {
            idx_t i, ii, nvtxs, ncon, bestcut = 0, inbfs, qnum;
            std::vector<idx_t> bestwhere, perm, counts;
            idx_t* vwgt, * where;

            WCOREPUSH;

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            vwgt = graph->vwgt.data();

            Allocate2WayPartitionMemory(ctrl, graph);
            where = graph->where.data();

            bestwhere.resize(nvtxs);
            perm.resize(nvtxs);
            counts.resize(ncon);

            for (inbfs = 0; inbfs < 2 * niparts; inbfs++) {
                irandArrayPermute(nvtxs, perm.data(), nvtxs / 2, 1);
                iset(ncon, 0, counts.data());

                /* partition by splitting the queues randomly */
                for (ii = 0; ii < nvtxs; ii++) {
                    i = perm[ii];
                    qnum = (idx_t)iargmax(ncon, vwgt + i * ncon, 1);
                    where[i] = (counts[qnum]++) % 2;
                }

                Compute2WayPartitionParams(ctrl, graph);

                FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
                Balance2Way(ctrl, graph, ntpwgts);
                FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
                Balance2Way(ctrl, graph, ntpwgts);
                FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

                if (inbfs == 0 || bestcut >= graph->mincut) {
                    bestcut = graph->mincut;
                    icopy(nvtxs, where, bestwhere.data());
                    if (bestcut == 0)
                        break;
                }
            }

            graph->mincut = bestcut;
            icopy(nvtxs, bestwhere.data(), where);

            WCOREPOP;
        }


        /*************************************************************************/
        /*! This function takes a multi-constraint graph and produces a bisection
            by using a region growing algorithm. The resulting partition is
            returned in graph->where.
        */
        /*************************************************************************/
        void McGrowBisection(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts,
            idx_t niparts)
        {
            idx_t nvtxs, bestcut = 0, inbfs;
            idx_t* where;
            std::vector<idx_t> bestwhere;

            WCOREPUSH;

            nvtxs = graph->nvtxs;

            Allocate2WayPartitionMemory(ctrl, graph);
            where = graph->where.data();

            bestwhere.resize(nvtxs);

            for (inbfs = 0; inbfs < 2 * niparts; inbfs++) {
                iset(nvtxs, 1, where);
                where[irandInRange(nvtxs)] = 0;

                Compute2WayPartitionParams(ctrl, graph);

                Balance2Way(ctrl, graph, ntpwgts);
                FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
                Balance2Way(ctrl, graph, ntpwgts);
                FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

                if (inbfs == 0 || bestcut >= graph->mincut) {
                    bestcut = graph->mincut;
                    icopy(nvtxs, where, bestwhere.data());
                    if (bestcut == 0)
                        break;
                }
            }

            graph->mincut = bestcut;
            icopy(nvtxs, bestwhere.data(), where);

            WCOREPOP;
        }

        void Init2WayPartition(ctrl_t* ctrl, graph_t* graph, real_t* ntpwgts,
            idx_t niparts)
        {
            mdbglvl_et dbglvl;

            ASSERT(graph->tvwgt[0] >= 0);

            dbglvl = ctrl->dbglvl;
            //IFSET(ctrl->dbglvl, METIS_DBG_REFINE, ctrl->dbglvl -= METIS_DBG_REFINE);
            //IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, ctrl->dbglvl -= METIS_DBG_MOVEINFO);

            //IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->InitPartTmr));

            switch (ctrl->iptype) {
            case METIS_IPTYPE_RANDOM:
                if (graph->ncon == 1)
                    RandomBisection(ctrl, graph, ntpwgts, niparts);
                else
                    McRandomBisection(ctrl, graph, ntpwgts, niparts);
                break;

            case METIS_IPTYPE_GROW:
                if (graph->nedges == 0)
                    if (graph->ncon == 1)
                        RandomBisection(ctrl, graph, ntpwgts, niparts);
                    else
                        McRandomBisection(ctrl, graph, ntpwgts, niparts);
                else
                    if (graph->ncon == 1)
                        GrowBisection(ctrl, graph, ntpwgts, niparts);
                    else
                        McGrowBisection(ctrl, graph, ntpwgts, niparts);
                break;

            default:
                gk_errexit("Unknown initial partition type: %d\n");
            }

            ctrl->dbglvl = dbglvl;

        }

#define METIS_NOPTIONS          40
        void InitKWayPartitioning(ctrl_t* ctrl, graph_t* graph)
        {
            idx_t i, options[METIS_NOPTIONS], curobj = 0;
            std::vector<real_t> ubvec;
            int status;

            iset(METIS_NOPTIONS, -1, options);

            //options[METIS_OPTION_NITER]     = 10;
            options[METIS_OPTION_NITER] = ctrl->niter;
            options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT;
            options[METIS_OPTION_NO2HOP] = ctrl->no2hop;
            options[METIS_OPTION_DROPEDGES] = ctrl->dropedges;
            //options[METIS_OPTION_DBGLVL]    = ctrl->dbglvl;

            ubvec.resize(graph->ncon);
            for (i = 0; i < graph->ncon; i++)
                ubvec[i] = (real_t)powf(ctrl->ubfactors[i], 1.0f / logf((float)ctrl->nparts));


            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
            case METIS_OBJTYPE_VOL:
                options[METIS_OPTION_NCUTS] = ctrl->nIparts;
                status = METIS_PartGraphRecursive(&graph->nvtxs, &graph->ncon,
                    graph->xadj.data(), graph->adjncy.data(), graph->vwgt.data(), graph->vsize.data(),
                    graph->adjwgt.data(), &ctrl->nparts, ctrl->tpwgts.data(), ubvec.data(),
                    options, &curobj, graph->where.data(), m_error_hander);

                if (status != METIS_OK)
                    gk_errexit("Failed during initial partitioning\n");

                break;

            default:
                gk_errexit("Unknown objtype: %d\n");
            }
        }

        void AllocateKWayPartitionMemory(ctrl_t* ctrl, graph_t* graph)
        {
            graph->pwgts.resize(ctrl->nparts * graph->ncon);
            graph->where.resize(graph->nvtxs);
            graph->bndptr.resize(graph->nvtxs);
            graph->bndind.resize(graph->nvtxs);

            switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
                graph->ckrinfo.resize(graph->nvtxs);
                break;

            case METIS_OBJTYPE_VOL:
                graph->vkrinfo.resize(graph->nvtxs);

                /* This is to let the cut-based -minconn and -contig large-scale graph
                   changes to go through */
                graph->ckrinfo.resize(graph->nvtxs);
                break;

            default:
                gk_errexit("Unknown objtype");
            }

        }

        idx_t MultilevelBisect(ctrl_t* ctrl, graph_t* graph, real_t* tpwgts)
        {
            idx_t i, niparts, bestobj = 0, curobj = 0, * bestwhere = NULL;
            graph_t* cgraph;
            real_t bestbal = 0.0, curbal = 0.0;
            std::vector<idx_t> v_bestwhere;

            Setup2WayBalMultipliers(ctrl, graph, tpwgts);

            WCOREPUSH;

            if (ctrl->ncuts > 1)
            {
                v_bestwhere.resize(graph->nvtxs);
                bestwhere = v_bestwhere.data();
            }

            for (i = 0; i < ctrl->ncuts; i++) {
                cgraph = CoarsenGraph(ctrl, graph);

                niparts = (cgraph->nvtxs <= ctrl->CoarsenTo ? SMALLNIPARTS : LARGENIPARTS);
                Init2WayPartition(ctrl, cgraph, tpwgts, niparts);

                Refine2Way(ctrl, graph, cgraph, tpwgts);

                curobj = graph->mincut;
                curbal = ComputeLoadImbalanceDiff(graph, 2, ctrl->pijbm.data(), ctrl->ubfactors.data());

                if (i == 0
                    || (curbal <= 0.0005 && bestobj > curobj)
                    || (bestbal > 0.0005 && curbal < bestbal)) {
                    bestobj = curobj;
                    bestbal = curbal;
                    if (i < ctrl->ncuts - 1)
                        icopy(graph->nvtxs, graph->where.data(), bestwhere);
                }

                if (bestobj == 0)
                    break;

                if (i < ctrl->ncuts - 1)
                    FreeRData(graph);
            }

            if (bestobj != curobj) {
                icopy(graph->nvtxs, bestwhere, graph->where.data());
                Compute2WayPartitionParams(ctrl, graph);
            }

            WCOREPOP;

            return bestobj;
        }


        graph_t* SetupSplitGraph(graph_t* graph, idx_t snvtxs, idx_t snedges)
        {
            graph_t* sgraph = CreateGraph();

            sgraph->nvtxs = snvtxs;
            sgraph->nedges = snedges;
            sgraph->ncon = graph->ncon;

            /* Allocate memory for the split graph */
            sgraph->xadj.resize(snvtxs + 1);
            sgraph->vwgt.resize(sgraph->ncon * snvtxs);
            sgraph->adjncy.resize(snedges);
            sgraph->adjwgt.resize(snedges);
            sgraph->label.resize(snvtxs);
            sgraph->tvwgt.resize(sgraph->ncon);
            sgraph->invtvwgt.resize(sgraph->ncon);

            if (graph->vsize.size() > 0)
                sgraph->vsize.resize(snvtxs);

            return sgraph;
        }


        void SplitGraphPart(ctrl_t* ctrl, graph_t* graph, graph_t** r_lgraph,
            graph_t** r_rgraph)
        {
            idx_t i, j, k, l, istart, iend, mypart, nvtxs, ncon, snvtxs[2], snedges[2];
            idx_t* xadj, * vwgt, * adjncy, * adjwgt, * label, * where, * bndptr;
            idx_t* sxadj[2], * svwgt[2], * sadjncy[2], * sadjwgt[2], * slabel[2];
            std::vector<idx_t> rename;
            idx_t* auxadjncy, * auxadjwgt;
            graph_t* lgraph, * rgraph;

            WCOREPUSH;

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->SplitTmr));

            nvtxs = graph->nvtxs;
            ncon = graph->ncon;
            xadj = graph->xadj.data();
            vwgt = graph->vwgt.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();
            label = graph->label.data();
            where = graph->where.data();
            bndptr = graph->bndptr.data();

            assert(bndptr != NULL);

            rename.resize(nvtxs);

            snvtxs[0] = snvtxs[1] = snedges[0] = snedges[1] = 0;
            for (i = 0; i < nvtxs; i++) {
                k = where[i];
                rename[i] = snvtxs[k]++;
                snedges[k] += xadj[i + 1] - xadj[i];
            }

            lgraph = SetupSplitGraph(graph, snvtxs[0], snedges[0]);
            sxadj[0] = lgraph->xadj.data();
            svwgt[0] = lgraph->vwgt.data();
            sadjncy[0] = lgraph->adjncy.data();
            sadjwgt[0] = lgraph->adjwgt.data();
            slabel[0] = lgraph->label.data();

            rgraph = SetupSplitGraph(graph, snvtxs[1], snedges[1]);
            sxadj[1] = rgraph->xadj.data();
            svwgt[1] = rgraph->vwgt.data();
            sadjncy[1] = rgraph->adjncy.data();
            sadjwgt[1] = rgraph->adjwgt.data();
            slabel[1] = rgraph->label.data();

            snvtxs[0] = snvtxs[1] = snedges[0] = snedges[1] = 0;
            sxadj[0][0] = sxadj[1][0] = 0;
            for (i = 0; i < nvtxs; i++) {
                mypart = where[i];

                istart = xadj[i];
                iend = xadj[i + 1];
                if (bndptr[i] == -1) { /* This is an interior vertex */
                    auxadjncy = sadjncy[mypart] + snedges[mypart] - istart;
                    auxadjwgt = sadjwgt[mypart] + snedges[mypart] - istart;
                    for (j = istart; j < iend; j++) {
                        auxadjncy[j] = adjncy[j];
                        auxadjwgt[j] = adjwgt[j];
                    }
                    snedges[mypart] += iend - istart;
                }
                else {
                    auxadjncy = sadjncy[mypart];
                    auxadjwgt = sadjwgt[mypart];
                    l = snedges[mypart];
                    for (j = istart; j < iend; j++) {
                        k = adjncy[j];
                        if (where[k] == mypart) {
                            auxadjncy[l] = k;
                            auxadjwgt[l++] = adjwgt[j];
                        }
                    }
                    snedges[mypart] = l;
                }

                /* copy vertex weights */
                for (k = 0; k < ncon; k++)
                    svwgt[mypart][snvtxs[mypart] * ncon + k] = vwgt[i * ncon + k];

                slabel[mypart][snvtxs[mypart]] = label[i];
                sxadj[mypart][++snvtxs[mypart]] = snedges[mypart];
            }

            for (mypart = 0; mypart < 2; mypart++) {
                iend = sxadj[mypart][snvtxs[mypart]];
                auxadjncy = sadjncy[mypart];
                for (i = 0; i < iend; i++)
                    auxadjncy[i] = rename[auxadjncy[i]];
            }

            lgraph->nedges = snedges[0];
            rgraph->nedges = snedges[1];

            SetupGraph_tvwgt(lgraph);
            SetupGraph_tvwgt(rgraph);

            // IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->SplitTmr));

            *r_lgraph = lgraph;
            *r_rgraph = rgraph;

            WCOREPOP;
        }

    private:
        ctrl_t* SetupCtrl(moptype_et optype, idx_t* options, idx_t ncon, idx_t nparts,
            real_t* tpwgts, real_t* ubvec)
        {
            idx_t i, j;
            ctrl_t* ctrl;

            ctrl = new ctrl_t;
            ctrl->optype = (moptype_et)0;
            ctrl->objtype = (mobjtype_et)0;
            ctrl->dbglvl = (mdbglvl_et)0;
            ctrl->ctype = (mctype_et)0;
            ctrl->iptype = (miptype_et)0;

            ctrl->CoarsenTo = 0;
            ctrl->nIparts = 0;
            ctrl->no2hop = 0;
            ctrl->minconn = 0;
            ctrl->contig = 0;
            ctrl->nseps = 0;
            ctrl->ufactor = 0;
            ctrl->compress = 0;
            ctrl->ccorder = 0;
            ctrl->seed = 0;
            ctrl->ncuts = 0;
            ctrl->niter = 0;
            ctrl->numflag = 0;
            ctrl->dropedges = 0;
            ctrl->ncon = 0;
            ctrl->nparts = 0;

            ctrl->pfactor = 0.0f;
            ctrl->cfactor = 0.0f;

            ctrl->nbrpoolsize_max = 0;
            ctrl->nbrpoolsize = 0;
            ctrl->nbrpoolcpos = 0;
            ctrl->nbrpoolreallocs = 0;

            switch (optype) {
            case METIS_OP_PMETIS:
                ctrl->objtype = (mobjtype_et)GETOPTION(options, METIS_OPTION_OBJTYPE, METIS_OBJTYPE_CUT);
                ctrl->ncuts = GETOPTION(options, METIS_OPTION_NCUTS, 1);
                ctrl->niter = GETOPTION(options, METIS_OPTION_NITER, 10);

                if (ncon == 1) {
                    ctrl->iptype = (miptype_et)GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_GROW);
                    ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, PMETIS_DEFAULT_UFACTOR);
                    ctrl->CoarsenTo = 20;
                }
                else {
                    ctrl->iptype = (miptype_et)GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_RANDOM);
                    ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, MCPMETIS_DEFAULT_UFACTOR);
                    ctrl->CoarsenTo = 100;
                }

                break;


            case METIS_OP_KMETIS:
                ctrl->objtype = (mobjtype_et)GETOPTION(options, METIS_OPTION_OBJTYPE, METIS_OBJTYPE_CUT);
                ctrl->iptype = (miptype_et)GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_METISRB);
                ctrl->nIparts = GETOPTION(options, METIS_OPTION_NIPARTS, -1);
                ctrl->ncuts = GETOPTION(options, METIS_OPTION_NCUTS, 1);
                ctrl->niter = GETOPTION(options, METIS_OPTION_NITER, 10);
                ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, KMETIS_DEFAULT_UFACTOR);
                ctrl->minconn = GETOPTION(options, METIS_OPTION_MINCONN, 0);
                ctrl->contig = GETOPTION(options, METIS_OPTION_CONTIG, 0);
                break;


            case METIS_OP_OMETIS:
                ctrl->objtype = (mobjtype_et)GETOPTION(options, METIS_OPTION_OBJTYPE, METIS_OBJTYPE_NODE);
                ctrl->iptype = (miptype_et)GETOPTION(options, METIS_OPTION_IPTYPE, METIS_IPTYPE_EDGE);
                ctrl->nseps = GETOPTION(options, METIS_OPTION_NSEPS, 1);
                ctrl->niter = GETOPTION(options, METIS_OPTION_NITER, 10);
                ctrl->ufactor = GETOPTION(options, METIS_OPTION_UFACTOR, OMETIS_DEFAULT_UFACTOR);
                ctrl->compress = GETOPTION(options, METIS_OPTION_COMPRESS, 1);
                ctrl->ccorder = GETOPTION(options, METIS_OPTION_CCORDER, 0);
                ctrl->pfactor = 0.1f * GETOPTION(options, METIS_OPTION_PFACTOR, 0);

                ctrl->CoarsenTo = 100;
                break;

            default:
                gk_errexit("Unknown optype of %d\n");
            }

            /* common options */
            ctrl->ctype = (mctype_et)GETOPTION(options, METIS_OPTION_CTYPE, METIS_CTYPE_SHEM);
            ctrl->no2hop = GETOPTION(options, METIS_OPTION_NO2HOP, 0);
            ctrl->seed = GETOPTION(options, METIS_OPTION_SEED, -1);
            ctrl->dbglvl = (mdbglvl_et)GETOPTION(options, METIS_OPTION_DBGLVL, 0);
            ctrl->numflag = GETOPTION(options, METIS_OPTION_NUMBERING, 0);
            ctrl->dropedges = GETOPTION(options, METIS_OPTION_DROPEDGES, 0);

            /* set non-option information */
            ctrl->optype = optype;
            ctrl->ncon = ncon;
            ctrl->nparts = nparts;
            ctrl->maxvwgt.resize(ncon, 0);

            /* setup the target partition weights */
            if (ctrl->optype != METIS_OP_OMETIS) {
                ctrl->tpwgts.resize(nparts * ncon, 0.0);
                if (tpwgts) {
                    rcopy(nparts * ncon, tpwgts, ctrl->tpwgts.data());
                }
                else {
                    for (i = 0; i < nparts; i++) {
                        for (j = 0; j < ncon; j++)
                            ctrl->tpwgts[i * ncon + j] = 1.0f / nparts;
                    }
                }
            }
            else {  /* METIS_OP_OMETIS */
              /* this is required to allow the pijbm to be defined properly for
                 the edge-based refinement during initial partitioning */
                ctrl->tpwgts.resize(2, 0.5f);
            }


            /* setup the ubfactors */
            ctrl->ubfactors.resize(ctrl->ncon, I2RUBFACTOR(ctrl->ufactor));
            if (ubvec)
                rcopy(ctrl->ncon, ubvec, ctrl->ubfactors.data());
            for (i = 0; i < ctrl->ncon; i++)
                ctrl->ubfactors[i] += 0.0000499f;

            /* Allocate memory for balance multipliers.
               Note that for PMETIS/OMETIS routines the memory allocated is more
               than required as balance multipliers for 2 parts is sufficient. */
            ctrl->pijbm.resize(nparts * ncon);

            InitRandom(ctrl->seed);

            // IFSET(ctrl->dbglvl, METIS_DBG_INFO, PrintCtrl(ctrl));

            if (!CheckParams(ctrl)) {
                delete ctrl;
                return NULL;
            }
            else {
                return ctrl;
            }

            return ctrl;
        }

        int CheckParams(ctrl_t* ctrl)
        {
            idx_t i, j;
            real_t sum;
            mdbglvl_et  dbglvl = METIS_DBG_INFO;

            switch (ctrl->optype) {
            case METIS_OP_PMETIS:
                if (ctrl->objtype != METIS_OBJTYPE_CUT) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect objective type.\n"));
                    return 0;
                }
                if (ctrl->ctype != METIS_CTYPE_RM && ctrl->ctype != METIS_CTYPE_SHEM) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect coarsening scheme.\n"));
                    return 0;
                }
                if (ctrl->iptype != METIS_IPTYPE_GROW && ctrl->iptype != METIS_IPTYPE_RANDOM) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect initial partitioning scheme.\n"));
                    return 0;
                }
                if (ctrl->ncuts <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncuts.\n"));
                    return 0;
                }
                if (ctrl->niter <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect niter.\n"));
                    return 0;
                }
                if (ctrl->ufactor <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ufactor.\n"));
                    return 0;
                }
                if (ctrl->numflag != 0 && ctrl->numflag != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect numflag.\n"));
                    return 0;
                }
                if (ctrl->nparts <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nparts.\n"));
                    return 0;
                }
                if (ctrl->ncon <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncon.\n"));
                    return 0;
                }

                for (i = 0; i < ctrl->ncon; i++) {
                    sum = _sum<float>(ctrl->nparts, ctrl->tpwgts.data() + i, ctrl->ncon);
                    if (sum < 0.99 || sum > 1.01) {
                        //IFSET(dbglvl, METIS_DBG_INFO,
                        //	printf("Input Error: Incorrect sum of %"PRREAL" for tpwgts for constraint %"PRIDX".\n", sum, i));
                        return 0;
                    }
                }
                for (i = 0; i < ctrl->ncon; i++) {
                    for (j = 0; j < ctrl->nparts; j++) {
                        if (ctrl->tpwgts[j * ctrl->ncon + i] <= 0.0) {
                            //IFSET(dbglvl, METIS_DBG_INFO,
                            //	printf("Input Error: Incorrect tpwgts for partition %"PRIDX" and constraint %"PRIDX".\n", j, i));
                            return 0;
                        }
                    }
                }

                for (i = 0; i < ctrl->ncon; i++) {
                    if (ctrl->ubfactors[i] <= 1.0) {
                        //IFSET(dbglvl, METIS_DBG_INFO,
                        //	printf("Input Error: Incorrect ubfactor for constraint %"PRIDX".\n", i));
                        return 0;
                    }
                }

                break;

            case METIS_OP_KMETIS:
                if (ctrl->objtype != METIS_OBJTYPE_CUT && ctrl->objtype != METIS_OBJTYPE_VOL) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect objective type.\n"));
                    return 0;
                }
                if (ctrl->ctype != METIS_CTYPE_RM && ctrl->ctype != METIS_CTYPE_SHEM) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect coarsening scheme.\n"));
                    return 0;
                }
                if (ctrl->iptype != METIS_IPTYPE_METISRB && ctrl->iptype != METIS_IPTYPE_GROW) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect initial partitioning scheme.\n"));
                    return 0;
                }
                if (ctrl->ncuts <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncuts.\n"));
                    return 0;
                }
                if (ctrl->niter <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect niter.\n"));
                    return 0;
                }
                if (ctrl->ufactor <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ufactor.\n"));
                    return 0;
                }
                if (ctrl->numflag != 0 && ctrl->numflag != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect numflag.\n"));
                    return 0;
                }
                if (ctrl->nparts <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nparts.\n"));
                    return 0;
                }
                if (ctrl->ncon <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncon.\n"));
                    return 0;
                }
                if (ctrl->contig != 0 && ctrl->contig != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect contig.\n"));
                    return 0;
                }
                if (ctrl->minconn != 0 && ctrl->minconn != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect minconn.\n"));
                    return 0;
                }

                for (i = 0; i < ctrl->ncon; i++) {
                    sum = _sum<float>(ctrl->nparts, ctrl->tpwgts.data() + i, ctrl->ncon);
                    if (sum < 0.99 || sum > 1.01) {
                        //IFSET(dbglvl, METIS_DBG_INFO,
                        //	printf("Input Error: Incorrect sum of %"PRREAL" for tpwgts for constraint %"PRIDX".\n", sum, i));
                        return 0;
                    }
                }
                for (i = 0; i < ctrl->ncon; i++) {
                    for (j = 0; j < ctrl->nparts; j++) {
                        if (ctrl->tpwgts[j * ctrl->ncon + i] <= 0.0) {
                            //IFSET(dbglvl, METIS_DBG_INFO,
                            //	printf("Input Error: Incorrect tpwgts for partition %"PRIDX" and constraint %"PRIDX".\n", j, i));
                            return 0;
                        }
                    }
                }

                for (i = 0; i < ctrl->ncon; i++) {
                    if (ctrl->ubfactors[i] <= 1.0) {
                        //IFSET(dbglvl, METIS_DBG_INFO,
                        //	printf("Input Error: Incorrect ubfactor for constraint %"PRIDX".\n", i));
                        return 0;
                    }
                }

                break;



            case METIS_OP_OMETIS:
                if (ctrl->objtype != METIS_OBJTYPE_NODE) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect objective type.\n"));
                    return 0;
                }
                if (ctrl->ctype != METIS_CTYPE_RM && ctrl->ctype != METIS_CTYPE_SHEM) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect coarsening scheme.\n"));
                    return 0;
                }
                if (ctrl->iptype != METIS_IPTYPE_EDGE && ctrl->iptype != METIS_IPTYPE_NODE) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect initial partitioning scheme.\n"));
                    return 0;
                }
                if (ctrl->nseps <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nseps.\n"));
                    return 0;
                }
                if (ctrl->niter <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect niter.\n"));
                    return 0;
                }
                if (ctrl->ufactor <= 0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ufactor.\n"));
                    return 0;
                }
                if (ctrl->numflag != 0 && ctrl->numflag != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect numflag.\n"));
                    return 0;
                }
                if (ctrl->nparts != 3) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect nparts.\n"));
                    return 0;
                }
                if (ctrl->ncon != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ncon.\n"));
                    return 0;
                }
                if (ctrl->compress != 0 && ctrl->compress != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect compress.\n"));
                    return 0;
                }
                if (ctrl->ccorder != 0 && ctrl->ccorder != 1) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect ccorder.\n"));
                    return 0;
                }
                if (ctrl->pfactor < 0.0) {
                    IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect pfactor.\n"));
                    return 0;
                }

                for (i = 0; i < ctrl->ncon; i++) {
                    if (ctrl->ubfactors[i] <= 1.0) {
                        //IFSET(dbglvl, METIS_DBG_INFO,
                        //	printf("Input Error: Incorrect ubfactor for constraint %"PRIDX".\n", i));
                        return 0;
                    }
                }

                break;

            default:
                IFSET(dbglvl, METIS_DBG_INFO, printf("Input Error: Incorrect optype\n"));
                return 0;
            }

            return 1;
        }

        void InitGraph(graph_t* graph)
        {
            graph->nvtxs = -1;
            graph->nedges = -1;
            graph->ncon = -1;
            graph->mincut = -1;
            graph->minvol = -1;
            graph->nbnd = -1;
            graph->droppedewgt = 0;

            graph->xadj.clear();
            graph->vwgt.clear();
            graph->vsize.clear();
            graph->adjncy.clear();
            graph->adjwgt.clear();
            graph->label.clear();
            graph->cmap.clear();
            graph->tvwgt.clear();
            graph->invtvwgt.clear();

            graph->where.clear();
            graph->pwgts.clear();
            graph->id.clear();
            graph->ed.clear();
            graph->bndptr.clear();
            graph->bndind.clear();
            graph->nrinfo.clear();
            graph->ckrinfo.clear();
            graph->vkrinfo.clear();

            graph->coarser = NULL;
            graph->finer = NULL;
        }

        graph_t* CreateGraph()
        {
            graph_t* graph = new graph_t;

            InitGraph(graph);

            return graph;
        }

        void SetupGraph_tvwgt(graph_t* graph)
        {
            idx_t i;

            if (graph->tvwgt.empty())
                graph->tvwgt.resize(graph->ncon);
            if (graph->invtvwgt.empty())
                graph->invtvwgt.resize(graph->ncon);

            for (i = 0; i < graph->ncon; i++) {
                graph->tvwgt[i] = _sum<int>(graph->nvtxs, graph->vwgt.data() + i, graph->ncon);
                graph->invtvwgt[i] = 1.0f / (graph->tvwgt[i] > 0 ? graph->tvwgt[i] : 1);
            }
        }

        void SetupGraph_label(graph_t* graph)
        {
            idx_t i;

            if (graph->label.empty())
                graph->label.resize(graph->nvtxs);

            for (i = 0; i < graph->nvtxs; i++)
                graph->label[i] = i;
        }

        int CheckGraph(graph_t* graph, int numflag, int verbose)
        {
            idx_t i, j, k, l;
            idx_t nvtxs, err = 0;
            idx_t minedge, maxedge, minewgt, maxewgt;
            idx_t* xadj, * adjncy, * adjwgt;
            std::vector<idx_t> htable;

            numflag = (numflag == 0 ? 0 : 1);  /* make sure that numflag is 0 or 1 */

            nvtxs = graph->nvtxs;
            xadj = graph->xadj.data();
            adjncy = graph->adjncy.data();
            adjwgt = graph->adjwgt.data();

            htable.resize(nvtxs, 0);

            if (graph->nedges > 0) {
                minedge = maxedge = adjncy[0];
                if (adjwgt)
                    minewgt = maxewgt = adjwgt[0];
            }

            for (i = 0; i < nvtxs; i++) {
                for (j = xadj[i]; j < xadj[i + 1]; j++) {
                    k = adjncy[j];

                    minedge = (k < minedge) ? k : minedge;
                    maxedge = (k > maxedge) ? k : maxedge;
                    if (adjwgt) {
                        minewgt = (adjwgt[j] < minewgt) ? adjwgt[j] : minewgt;
                        maxewgt = (adjwgt[j] > maxewgt) ? adjwgt[j] : maxewgt;
                    }

                    if (i == k) {
                        if (verbose)
                            printf("Vertex %d contains a self-loop "
                                "(i.e., diagonal entry in the matrix)!\n", i + numflag);
                        err++;
                    }
                    else {
                        for (l = xadj[k]; l < xadj[k + 1]; l++) {
                            if (adjncy[l] == i) {
                                if (adjwgt) {
                                    if (adjwgt[l] != adjwgt[j]) {
                                        if (verbose)
                                            printf("Edges (u:%d v:%d wgt:%d) and "
                                                "(v:%d u:%d wgt:%d) "
                                                "do not have the same weight!\n",
                                                i + numflag, k + numflag, adjwgt[j],
                                                k + numflag, i + numflag, adjwgt[l]);
                                        err++;
                                    }
                                }
                                break;
                            }
                        }
                        if (l == xadj[k + 1]) {
                            if (verbose)
                                printf("Missing edge: (%d %d)!\n", k + numflag, i + numflag);
                            err++;
                        }
                    }

                    if (htable[k] == 0) {
                        htable[k]++;
                    }
                    else {
                        if (verbose)
                            printf("Edge %d from vertex %d is repeated %d times\n",
                                k + numflag, i + numflag, htable[k]++);
                        err++;
                    }
                }

                for (j = xadj[i]; j < xadj[i + 1]; j++)
                    htable[adjncy[j]] = 0;
            }


            if (err > 0 && verbose) {
                printf("A total of %d errors exist in the input file. "
                    "Correct them, and run again!\n", err);
            }

            return (err == 0 ? 1 : 0);
        }


        graph_t* SetupGraph(ctrl_t* ctrl, idx_t nvtxs, idx_t ncon, idx_t* xadj,
            idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt)
        {
            idx_t i, j;
            graph_t* graph = nullptr;

            graph = CreateGraph();

            graph->nvtxs = nvtxs;
            graph->nedges = xadj[nvtxs];
            graph->ncon = ncon;

            graph->xadj.resize(nvtxs + 1);
            icopy(graph->xadj.size(), xadj, graph->xadj.data());

            graph->adjncy.resize(graph->nedges);
            icopy(graph->adjncy.size(), adjncy, graph->adjncy.data());

            graph->droppedewgt = 0;

            /* setup the vertex weights */
            if (vwgt) {
                graph->vwgt.resize(ncon * nvtxs);
                icopy(ncon * nvtxs, vwgt, graph->vwgt.data());
            }
            else {
                graph->vwgt.resize(ncon * nvtxs, 1);
                vwgt = graph->vwgt.data();
            }

            graph->tvwgt.resize(ncon);
            graph->invtvwgt.resize(ncon);
            for (i = 0; i < ncon; i++) {
                graph->tvwgt[i] = _sum<int>(nvtxs, vwgt + i, ncon);
                graph->invtvwgt[i] = 1.0f / (graph->tvwgt[i] > 0 ? graph->tvwgt[i] : 1);
            }


            if (ctrl->objtype == METIS_OBJTYPE_VOL) {
                /* Setup the vsize */
                if (vsize) {
                    graph->vsize.resize(nvtxs);
                    icopy(nvtxs, vsize, graph->vsize.data());
                }
                else {
                    graph->vsize.resize(nvtxs, 1);
                    vsize = graph->vsize.data();
                }

                /* Allocate memory for edge weights and initialize them to the sum of the vsize */
                graph->adjwgt.resize(graph->nedges);
                adjwgt = graph->adjwgt.data();
                for (i = 0; i < nvtxs; i++) {
                    for (j = xadj[i]; j < xadj[i + 1]; j++)
                        adjwgt[j] = 1 + vsize[i] + vsize[adjncy[j]];
                }
            }
            else { /* For edgecut minimization */
              /* setup the edge weights */
                if (adjwgt) {
                    graph->adjwgt.resize(graph->nedges);
                    icopy(graph->nedges, adjwgt, graph->adjwgt.data());
                }
                else {
                    graph->adjwgt.resize(graph->nedges, 1);
                    adjwgt = graph->adjwgt.data();
                }
            }

            /* setup various derived info */
            SetupGraph_tvwgt(graph);

            if (ctrl->optype == METIS_OP_PMETIS || ctrl->optype == METIS_OP_OMETIS)
                SetupGraph_label(graph);

            assert(CheckGraph(graph, ctrl->numflag, 1));

            return graph;
        }

        void FreeRData(graph_t* graph)
        {
            graph->where.clear();
            graph->pwgts.clear();
            graph->id.clear();
            graph->ed.clear();
            graph->bndptr.clear();
            graph->bndind.clear();
            graph->nrinfo.clear();
            graph->vkrinfo.clear();
            graph->ckrinfo.clear();
        }

    private:
        void gk_errexit(const char* error_msg)
        {
            m_status = METIS_ERROR;

            if (m_error_hander)
            {
                m_error_hander(error_msg);
            }
        }

    public:
        rstatus_et    m_status;
        error_handler m_error_hander;
    };

    // https://www.lrz.de/services/software/mathematik/metis/metis_5_0.pdf, Page.25
    int METIS_PartGraphRecursive(idx_t* nvtxs, idx_t* ncon, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt, idx_t* nparts, real_t* tpwgts, real_t* ubvec, idx_t* options, idx_t* edgecut, idx_t* part, error_handler h)
    {
        MetisGraphPartition gp;
        gp.m_error_hander = h;
        return gp.PartGraphRecursive(nvtxs, ncon, xadj, adjncy, vwgt, vsize, adjwgt, nparts, tpwgts, ubvec, options, edgecut, part);
    }

    // https://www.lrz.de/services/software/mathematik/metis/metis_5_0.pdf, Page.25
    int METIS_PartGraphKway(idx_t* nvtxs, idx_t* ncon, idx_t* xadj, idx_t* adjncy, idx_t* vwgt, idx_t* vsize, idx_t* adjwgt, idx_t* nparts, real_t* tpwgts, real_t* ubvec, idx_t* options, idx_t* edgecut, idx_t* part, error_handler h)
    {
        MetisGraphPartition gp;
        gp.m_error_hander = h;
        return gp.PartGraphKway(nvtxs, ncon, xadj, adjncy, vwgt, vsize, adjwgt, nparts, tpwgts, ubvec, options, edgecut, part);
    }
}

namespace Riemann
{
	void default_error_handler(const char* msg)
	{
	}

	static metis::idx_t NormalizeWeight(float strength, float scaler, metis::idx_t min_s, metis::idx_t max_s)
	{
		if (strength < 1e-3f)
		{
			return 1;
		}
		return std::max(min_s, std::min((metis::idx_t)(strength * 100.0f), max_s));
	}

	// https://www.lrz.de/services/software/mathematik/metis/metis_5_0.pdf, Page.22
	static bool BuildCsrFromConnectionGraph(const std::vector<int>& nodes, const ConnectionGraph* graph, std::map<int, int>& node_to_index, std::vector<metis::idx_t>& xadj, std::vector<metis::idx_t>& adjncy, std::vector<metis::idx_t>& adjwgt)
	{
		node_to_index.clear();
		for (size_t i = 0; i < nodes.size(); ++i)
		{
			node_to_index[nodes[i]] = (int)i;
		}

        const ListSet<Bond>& bonds = graph->GetBonds();
		for (size_t i = 0; i < bonds.size(); ++i)
		{
			uint32_t node = nodes[i];
			xadj.push_back((metis::idx_t)adjncy.size());
			for (const Bond& b : bonds[node])
			{
				adjncy.push_back(node_to_index[b.v0]);
				adjwgt.push_back(NormalizeWeight(b.strength, 100.0f, 1, 1000));
			}
		}
		xadj.push_back((metis::idx_t)adjncy.size());

		return true;
	}

	bool PartitionCsrGraph(const std::vector<int>& nodes, std::vector<int>& xadj, std::vector<int>& adjncy, std::vector<int>* adjwgt, int num_part, std::vector<std::vector<int>>& parts)
	{
        bool success = false;

        metis::idx_t nCon = 1;
        metis::idx_t objval;
        metis::idx_t nVertices = (metis::idx_t)nodes.size();
        std::vector<metis::idx_t> result(nodes.size(), 0);

        if (num_part >= 8)
        {
            int ret = metis::METIS_PartGraphKway(&nVertices, &nCon, xadj.data(), adjncy.data(),
                NULL, NULL, (adjwgt > 0 ? adjwgt->data() : NULL), &num_part, NULL,
                NULL, NULL, &objval, result.data(), default_error_handler);
            success = ret != 0;
        }
        else
        {
            int ret = metis::METIS_PartGraphRecursive(&nVertices, &nCon, xadj.data(), adjncy.data(),
                NULL, NULL, (adjwgt > 0 ? adjwgt->data() : NULL), &num_part, NULL,
                NULL, NULL, &objval, result.data(), default_error_handler);
            success = ret != 0;
        }

        if (!success)
        {
            return false;
        }

        parts.resize(num_part);
        for (size_t i = 0; i < result.size(); i++)
        {
            metis::idx_t part_idx = result[i];
            if (part_idx < 0 || part_idx >= (int)parts.size())
            {
                return false;
            }
            parts[part_idx].push_back(nodes[i]);
        }

        return true;
	}

    bool PartitionConnectionGraph(const std::vector<int>& nodes, const ConnectionGraph* p, int num_part, std::map<int, int>& node_to_index, std::vector<std::vector<int>>& parts)
    {
        std::vector<metis::idx_t> xadj;
        std::vector<metis::idx_t> adjncy;
        std::vector<metis::idx_t> adjwgt;

        if (!BuildCsrFromConnectionGraph(nodes, p, node_to_index, xadj, adjncy, adjwgt))
        {
            return false;
        }

        return PartitionCsrGraph(nodes, xadj, adjncy, &adjwgt, num_part, parts);
    }

}	// namespace Riemann