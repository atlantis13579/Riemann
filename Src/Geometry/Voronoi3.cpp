
#include <assert.h>
#include "Voronoi3.h"

namespace voro
{
	// These constants set the initial memory allocation for the Voronoi cell
	/* The initial memory allocation for the number of vertices. */
	const int init_vertices = 256;
	/* The initial memory allocation for the maximum vertex order. */
	const int init_vertex_order = 64;
	/* The initial memory allocation for the number of regular vertices of order
	 * 3. */
	const int init_3_vertices = 256;
	/* The initial memory allocation for the number of vertices of higher order.
	 */
	const int init_n_vertices = 8;
	/* The initial buffer size for marginal cases used by the suretest class. */
	const int init_marginal = 64;
	/* The initial size for the delete stack. */
	const int init_delete_size = 256;
	/* The initial size for the auxiliary delete stack. */
	const int init_delete2_size = 256;
	/* The initial size for the wall pointer array. */
	const int init_wall_size = 32;
	/* The default initial size for the ordering class. */
	const int init_ordering_size = 4096;
	/* The initial size of the pre_container chunk index. */
	const int init_chunk_size = 256;

	// If the initial memory is too small, the program dynamically allocates more.
	// However, if the limits below are reached, then the program bails out.
	/* The maximum memory allocation for the number of vertices. */
	const int max_vertices = 16777216;
	/* The maximum memory allocation for the maximum vertex order. */
	const int max_vertex_order = 16777216;
	/* The maximum memory allocation for the any particular order of vertex. */
	const int max_n_vertices = 16777216;
	/* The maximum buffer size for marginal cases used by the suretest class. */
	const int max_marginal = 16777216;
	/* The maximum size for the delete stack. */
	const int max_delete_size = 16777216;
	/* The maximum size for the auxiliary delete stack. */
	const int max_delete2_size = 16777216;
	/* The maximum amount of particle memory allocated for a single region. */
	const int max_particle_memory = 16777216;
	/* The maximum size for the wall pointer array. */
	const int max_wall_size = 2048;
	/* The maximum size for the ordering class. */
	const int max_ordering_size = 67108864;
	/* The maximum size for the pre_container chunk index. */
	const int max_chunk_size = 65536;

	/* The chunk size in the pre_container classes. */
	const int pre_container_chunk_size = 1024;

	/* Each region is divided into a grid of subregions, and a worklist is
	# constructed for each. This parameter sets is set to half the number of
	# subregions that the block is divided into. */
	const int wl_hgrid = 4;
	/* The number of subregions that a block is subdivided into, which is twice
	the value of hgrid. */
	const int wl_fgrid = 8;
	/* The total number of worklists, set to the cube of hgrid. */
	const int wl_hgridcu = 64;
	/* The number of elements in each worklist. */
	const int wl_seq_length = 64;

	/* If a point is within this distance of a cutting plane, then the code
	 * assumes that point exactly lies on the plane. */
	const double tolerance = 1e-11;

	/* If a point is within this distance of a cutting plane, then the code stores
	 * whether this point is inside, outside, or exactly on the cutting plane in
	 * the marginal cases buffer, to prevent the test giving a different result on
	 * a subsequent evaluation due to floating point rounding errors. */
	const double tolerance2 = 2e-11;

	/* The square of the tolerance, used when deciding whether some squared
	 * quantities are large enough to be used. */
	const double tolerance_sq = tolerance * tolerance;

	/* A large number that is used in the computation. */
	const double large_number = 1e30;

	/* A radius to use as a placeholder when no other information is available. */
	const double default_radius = 0.5;

	/* The maximum number of shells of periodic images to test over. */
	const int max_unit_voro_shells = 10;

	/* A guess for the optimal number of particles per block, used to set up the
	 * container grid. */
	const float optimal_particles = 5.6f;

	/* \brief Class containing data structures common across all particle container classes.
	 *
	 * This class contains constants and data structures that are common across all
	 * particle container classes. It contains constants setting the size of the
	 * underlying subgrid of blocks that forms the basis of the Voronoi cell
	 * computations. It also constructs bound tables that are used in the Voronoi
	 * cell computation, and contains a number of routines that are common across
	 * all container classes. */
	class voro_base {
	public:
		/* The number of blocks in the x direction. */
		const int nx;
		/* The number of blocks in the y direction. */
		const int ny;
		/* The number of blocks in the z direction. */
		const int nz;
		/* A constant, set to the value of nx multiplied by ny, which
		 * is used in the routines that step through blocks in
		 * sequence. */
		const int nxy;
		/* A constant, set to the value of nx*ny*nz, which is used in
		 * the routines that step through blocks in sequence. */
		const int nxyz;
		/* The size of a computational block in the x direction. */
		const double boxx;
		/* The size of a computational block in the y direction. */
		const double boxy;
		/* The size of a computational block in the z direction. */
		const double boxz;
		/* The inverse box length in the x direction. */
		const double xsp;
		/* The inverse box length in the y direction. */
		const double ysp;
		/* The inverse box length in the z direction. */
		const double zsp;
		/* An array to hold the minimum distances associated with the
		 * worklists. This array is initialized during container
		 * construction, by the initialize_radii() routine. */
		double* mrad;
		/* The pre-computed block worklists. */
		static const unsigned int wl[wl_seq_length * wl_hgridcu];
		bool contains_neighbor(const char* format) const
		{
			char* fmp = (const_cast<char*>(format));

			// Check to see if "%n" appears in the format sequence
			while (*fmp != 0) {
				if (*fmp == '%') {
					fmp++;
					if (*fmp == 'n') return true;
					else if (*fmp == 0) return false;
				}
				fmp++;
			}

			return false;
		}

		voro_base(int nx_, int ny_, int nz_, double boxx_, double boxy_, double boxz_)
		:	nx(nx_), ny(ny_), nz(nz_), nxy(nx_* ny_), nxyz(nxy* nz_), boxx(boxx_), boxy(boxy_), boxz(boxz_),
			xsp(1 / boxx_), ysp(1 / boxy_), zsp(1 / boxz_), mrad(new double[wl_hgridcu * wl_seq_length]) {
			const unsigned int b1 = 1 << 21, b2 = 1 << 22, b3 = 1 << 24, b4 = 1 << 25, b5 = 1 << 27, b6 = 1 << 28;
			const double xstep = boxx / wl_fgrid, ystep = boxy / wl_fgrid, zstep = boxz / wl_fgrid;
			int i, j, k, lx, ly, lz, q;
			unsigned int f, * e = const_cast<unsigned int*> (wl);
			double xlo, ylo, zlo, xhi, yhi, zhi, minr, * radp = mrad;
			for (zlo = 0, zhi = zstep, lz = 0; lz < wl_hgrid; zlo = zhi, zhi += zstep, lz++) {
				for (ylo = 0, yhi = ystep, ly = 0; ly < wl_hgrid; ylo = yhi, yhi += ystep, ly++) {
					for (xlo = 0, xhi = xstep, lx = 0; lx < wl_hgrid; xlo = xhi, xhi += xstep, lx++) {
						minr = large_number;
						for (q = e[0] + 1; q < wl_seq_length; q++) {
							f = e[q];
							i = (f & 127) - 64;
							j = (f >> 7 & 127) - 64;
							k = (f >> 14 & 127) - 64;
							if ((f & b2) == b2) {
								compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i - 1, j, k);
								if ((f & b1) == 0) compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i + 1, j, k);
							}
							else if ((f & b1) == b1) compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i + 1, j, k);
							if ((f & b4) == b4) {
								compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j - 1, k);
								if ((f & b3) == 0) compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j + 1, k);
							}
							else if ((f & b3) == b3) compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j + 1, k);
							if ((f & b6) == b6) {
								compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j, k - 1);
								if ((f & b5) == 0) compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j, k + 1);
							}
							else if ((f & b5) == b5) compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j, k + 1);
						}
						q--;
						while (q > 0) {
							radp[q] = minr;
							f = e[q];
							i = (f & 127) - 64;
							j = (f >> 7 & 127) - 64;
							k = (f >> 14 & 127) - 64;
							compute_minimum(minr, xlo, xhi, ylo, yhi, zlo, zhi, i, j, k);
							q--;
						}
						*radp = minr;
						e += wl_seq_length;
						radp += wl_seq_length;
					}
				}
			}
		}
		voro_base(const voro_base& Other) = delete;
		voro_base& operator=(const voro_base& Other) = delete;
		voro_base(voro_base&& Other) noexcept : nx(Other.nx), ny(Other.ny), nz(Other.nz), nxy(Other.nxy), nxyz(Other.nxyz), boxx(Other.boxx), boxy(Other.boxy), boxz(Other.boxz),
			xsp(Other.xsp), ysp(Other.ysp), zsp(Other.zsp), mrad(Other.mrad)
		{
			Other.mrad = nullptr;
		}
		// Note: Move assignment not implemented due to const members on this class (and it has not been needed yet)
		voro_base& operator=(voro_base&& Other) = delete;

		~voro_base() { delete[] mrad; }

	protected:
		/* A custom int function that returns consistent stepping
		 * for negative numbers, so that (-1.5, -0.5, 0.5, 1.5) maps
		 * to (-2,-1,0,1).
		 * \param[in] a the number to consider.
		 * \return The value of the custom int operation. */
		inline int step_int(double a) const { return a < 0 ? int(a) - 1 : int(a); }
		/* A custom modulo function that returns consistent stepping
		 * for negative numbers. For example, (-2,-1,0,1,2) step_mod 2
		 * is (0,1,0,1,0).
		 * \param[in] (a,b) the input integers.
		 * \return The value of a modulo b, consistent for negative
		 * numbers. */
		inline int step_mod(int a, int b) const { return a >= 0 ? a % b : b - 1 - (b - 1 - a) % b; }
		/* A custom integer division function that returns consistent
		 * stepping for negative numbers. For example, (-2,-1,0,1,2)
		 * step_div 2 is (-1,-1,0,0,1).
		 * \param[in] (a,b) the input integers.
		 * \return The value of a div b, consistent for negative
		 * numbers. */
		inline int step_div(int a, int b) const { return a >= 0 ? a / b : -1 + (a + 1) / b; }
	private:
		void compute_minimum(double& minr, double& xlo, double& xhi, double& ylo, double& yhi, double& zlo, double& zhi, int ti, int tj, int tk)
		{
			double radsq, temp;
			if (ti > 0) { temp = boxx * ti - xhi; radsq = temp * temp; }
			else if (ti < 0) { temp = xlo - boxx * (1 + ti); radsq = temp * temp; }
			else radsq = 0;

			if (tj > 0) { temp = boxy * tj - yhi; radsq += temp * temp; }
			else if (tj < 0) { temp = ylo - boxy * (1 + tj); radsq += temp * temp; }

			if (tk > 0) { temp = boxz * tk - zhi; radsq += temp * temp; }
			else if (tk < 0) { temp = zlo - boxz * (1 + tk); radsq += temp * temp; }

			if (radsq < minr) minr = radsq;
		}
	};

	const unsigned int voro_base::wl[wl_seq_length * wl_hgridcu] = {
	7,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0xfdfc0,0xfdfbf,0x10fe0bf,0x11020bf,0x11020c0,0x10fe0c0,0x2fe041,0x302041,0x301fc1,0x2fdfc1,0x8105fc0,0x8106040,0x810603f,0x8105fbf,0x701fbe,0x70203e,0x6fe03e,0x6fdfbe,0x30fdf3f,0x3101f3f,0x3101f40,0x30fdf40,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x12fe0c1,0x13020c1,0x91060c0,0x91060bf,0x8306041,0x8305fc1,0x3301f41,0x32fdf41,0x182f9fc1,0x182fa041,0x190fa0c0,0x190fa0bf,0x16fe0be,0x17020be,0x870603e,0x8705fbe,0xb105f3f,0xb105f40,0x3701f3e,0x36fdf3e,0x186f9fbe,0x186fa03e,0x1b0f9f3f,0x1b0f9f40,0x93060c1,0x192fa0c1,0x97060be,0xb305f41,0x1b2f9f41,0x196fa0be,0xb705f3e,0x1b6f9f3e,
	11,0x101fc0,0xfe040,0xfdfc0,0x10203f,0x101fbf,0xfe03f,0xfdfbf,0xfdfc1,0x101fc1,0x102041,0xfe041,0x10fe0c0,0x11020c0,0x8106040,0x8105fc0,0x8105fbf,0x810603f,0x11020bf,0x10fe0bf,0x180fa040,0x180f9fc0,0x30fdf40,0x3101f40,0x3101f3f,0x30fdf3f,0x180f9fbf,0x180fa03f,0x6fe03e,0x70203e,0x701fbe,0x6fdfbe,0x8105fc1,0x8106041,0x11020c1,0x10fe0c1,0x180fa041,0x180f9fc1,0x30fdf41,0x3101f41,0x91060c0,0x91060bf,0x190fa0c0,0x190fa0bf,0xb105f40,0xb105f3f,0x8705fbe,0x870603e,0x97020be,0x16fe0be,0x1b0f9f40,0x1b0f9f3f,0x36fdf3e,0xb701f3e,0x1b6f9fbe,0x196fa03e,0x93060c1,0xb305f41,0x192fa0c1,0x1b2f9f41,0x1b2fdfc2,0xb301fc2,0x9302042,0x192fe042,
	11,0x101fc0,0xfe040,0xfdfc0,0xfdfbf,0x101fbf,0x10203f,0xfe03f,0xfe041,0x102041,0x101fc1,0xfdfc1,0x8105fc0,0x8106040,0x11020c0,0x10fe0c0,0x10fe0bf,0x11020bf,0x810603f,0x8105fbf,0x3101f40,0x30fdf40,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x30fdf3f,0x3101f3f,0x8105fc1,0x8106041,0x11020c1,0x10fe0c1,0x180fa041,0x180f9fc1,0x30fdf41,0x3101f41,0x701fbe,0x70203e,0x6fe03e,0x6fdfbe,0x91060c0,0x91060bf,0xb105f40,0xb105f3f,0x190fa0c0,0x190fa0bf,0x93060c1,0x1b0f9f40,0x1b0f9f3f,0xb305f41,0x192fa0c1,0x16fe0be,0x17020be,0x970603e,0x8705fbe,0xb701f3e,0x36fdf3e,0x1b2f9f41,0x1b2fdfc2,0x192fe042,0x9302042,0xb301fc2,0x1b6f9fbe,0x196fa03e,
	11,0x101fc0,0xfe040,0xfdfc0,0xfdfbf,0x101fbf,0x10203f,0xfe03f,0xfe041,0x102041,0x101fc1,0xfdfc1,0x8105fc0,0x8106040,0x11020c0,0x10fe0c0,0x10fe0bf,0x11020bf,0x810603f,0x8105fbf,0x3101f40,0x30fdf40,0x180f9fc0,0x180fa040,0x10fe0c1,0x11020c1,0x8106041,0x8105fc1,0x3101f3f,0x30fdf3f,0x180f9fbf,0x180fa03f,0x180fa041,0x180f9fc1,0x30fdf41,0x3101f41,0x91060c0,0x91060bf,0x70203e,0x701fbe,0x6fdfbe,0x6fe03e,0x190fa0c0,0xb105f40,0xb105f3f,0x93060c1,0x190fa0bf,0x192fa0c1,0x1b0f9f40,0x1b0f9f3f,0xb305f41,0xb301fc2,0x9302042,0x192fe042,0x1b2fdfc2,0x1b2f9f41,0x16fe0be,0x17020be,0x970603e,0x8705fbe,0xb701f3e,0x36fdf3e,0x1b6f9fbe,0x196fa03e,
	11,0x10203f,0xfe040,0xfe03f,0x101fc0,0x101fbf,0xfdfc0,0xfdfbf,0xfe0bf,0x1020bf,0x1020c0,0xfe0c0,0x2fe041,0x302041,0x8106040,0x810603f,0x8105fbf,0x8105fc0,0x301fc1,0x2fdfc1,0x180fa040,0x180fa03f,0x6fe03e,0x70203e,0x701fbe,0x6fdfbe,0x180f9fbf,0x180f9fc0,0x30fdf40,0x3101f40,0x3101f3f,0x30fdf3f,0x81060bf,0x81060c0,0x3020c1,0x2fe0c1,0x180fa0c0,0x180fa0bf,0x6fe0be,0x7020be,0x8306041,0x8305fc1,0x182fa041,0x182f9fc1,0x870603e,0x8705fbe,0xb105f3f,0xb105f40,0xb301f41,0x32fdf41,0x186fa03e,0x186f9fbe,0x36fdf3e,0xb701f3e,0x1b6f9f3f,0x1b2f9f40,0x93060c1,0x97060be,0x192fa0c1,0x196fa0be,0x196fe13f,0x970213f,0x9302140,0x192fe140,
	9,0xfe040,0x10203f,0x101fc0,0xfdfc0,0xfe03f,0xfdfbf,0x101fbf,0x102041,0xfe041,0x10fe0c0,0x11020c0,0x11020bf,0x10fe0bf,0xfdfc1,0x101fc1,0x8106040,0x810603f,0x8105fc0,0x8105fbf,0x180fa040,0x180f9fc0,0x180fa03f,0x180f9fbf,0x10fe0c1,0x11020c1,0x70203e,0x6fe03e,0x6fdfbe,0x701fbe,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x8106041,0x91060c0,0x91060bf,0x8305fc1,0x180fa041,0x190fa0c0,0x190fa0bf,0x180f9fc1,0x30fdf41,0x3301f41,0x17020be,0x16fe0be,0x93060c1,0x870603e,0x8705fbe,0xb105f3f,0xb105f40,0x192fa0c1,0x1b0f9f40,0x1b0f9f3f,0x186f9fbe,0x196fa03e,0x1b6fdf3e,0xb701f3e,0x97060be,0xb305f41,0x1b2f9f41,0x1b2fdfc2,0x192fe042,0xa302042,
	11,0xfe040,0x101fc0,0xfdfc0,0xfe03f,0x10203f,0x101fbf,0xfdfbf,0xfe041,0x102041,0x101fc1,0xfdfc1,0x10fe0c0,0x11020c0,0x11020bf,0x10fe0bf,0x8106040,0x8105fc0,0x810603f,0x8105fbf,0x11020c1,0x10fe0c1,0x180fa040,0x180f9fc0,0x180fa03f,0x180f9fbf,0x30fdf40,0x3101f40,0x8106041,0x8105fc1,0x3101f3f,0x30fdf3f,0x91060c0,0x91060bf,0x180fa041,0x180f9fc1,0x6fe03e,0x70203e,0x701fbe,0x6fdfbe,0x190fa0c0,0x190fa0bf,0x30fdf41,0x3101f41,0x93060c1,0x192fa0c1,0xb105f40,0xb105f3f,0x17020be,0x16fe0be,0x970603e,0x8705fbe,0x1b0f9f40,0x1b0f9f3f,0x186f9fbe,0x196fa03e,0xb305f41,0xb301fc2,0x9302042,0x192fe042,0x1b2fdfc2,0x1b2f9f41,0x1b6fdf3e,0xb701f3e,
	11,0xfe040,0x101fc0,0xfdfc0,0xfe03f,0x10203f,0x101fbf,0xfdfbf,0xfe041,0x102041,0x101fc1,0xfdfc1,0x10fe0c0,0x11020c0,0x11020bf,0x10fe0bf,0x8106040,0x8105fc0,0x11020c1,0x10fe0c1,0x810603f,0x8105fbf,0x180fa040,0x180f9fc0,0x8106041,0x8105fc1,0x3101f40,0x180fa03f,0x180f9fbf,0x30fdf40,0x180fa041,0x180f9fc1,0x91060c0,0x3101f3f,0x30fdf3f,0x30fdf41,0x3101f41,0x91060bf,0x190fa0c0,0x91060c1,0x190fa0bf,0x186fe03e,0x70203e,0x3701fbe,0x1b6fdfbe,0x190fa0c1,0x182fe042,0xb105f40,0xb105f3f,0x302042,0x3301fc2,0x1b2fdfc2,0x1b0f9f40,0x1b6f9f3f,0xb105f41,0x17020be,0x196fe0be,0x970603e,0xb705fbe,0x1b2f9f41,0x192fe0c2,0x13020c2,0x9306042,0xb305fc2,
	11,0x10203f,0xfe040,0xfe03f,0xfdfbf,0x101fbf,0x101fc0,0xfdfc0,0xfe0c0,0x1020c0,0x1020bf,0xfe0bf,0x810603f,0x8106040,0x302041,0x2fe041,0x2fdfc1,0x301fc1,0x8105fc0,0x8105fbf,0x70203e,0x6fe03e,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x6fdfbe,0x701fbe,0x81060bf,0x81060c0,0x3020c1,0x2fe0c1,0x180fa0c0,0x180fa0bf,0x6fe0be,0x7020be,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x8306041,0x8305fc1,0x870603e,0x8705fbe,0x182fa041,0x182f9fc1,0x93060c1,0x186fa03e,0x186f9fbe,0x97060be,0x192fa0c1,0x32fdf41,0x3301f41,0xb305f40,0xb105f3f,0xb701f3e,0x36fdf3e,0x196fa0be,0x196fe13f,0x192fe140,0x9302140,0x970213f,0x1b6f9f3f,0x1b2f9f40,
	11,0xfe040,0x10203f,0xfe03f,0xfdfc0,0x101fc0,0x101fbf,0xfdfbf,0xfe0c0,0x1020c0,0x1020bf,0xfe0bf,0x2fe041,0x302041,0x301fc1,0x2fdfc1,0x8106040,0x810603f,0x8105fc0,0x8105fbf,0x3020c1,0x2fe0c1,0x180fa040,0x180fa03f,0x180f9fc0,0x180f9fbf,0x6fe03e,0x70203e,0x81060c0,0x81060bf,0x701fbe,0x6fdfbe,0x8306041,0x8305fc1,0x180fa0c0,0x180fa0bf,0x30fdf40,0x3101f40,0x3101f3f,0x30fdf3f,0x182fa041,0x182f9fc1,0x6fe0be,0x7020be,0x93060c1,0x192fa0c1,0x870603e,0x8705fbe,0x3301f41,0x32fdf41,0xb305f40,0xb105f3f,0x186fa03e,0x186f9fbe,0x1b0f9f3f,0x1b2f9f40,0x97060be,0x970213f,0x9302140,0x192fe140,0x196fe13f,0x196fa0be,0x1b6fdf3e,0xb701f3e,
	15,0xfe040,0xfe03f,0x10203f,0x101fc0,0xfdfc0,0xfdfbf,0x101fbf,0x102041,0xfe041,0xfe0c0,0x1020c0,0x1020bf,0xfe0bf,0xfdfc1,0x101fc1,0x8106040,0x1020c1,0xfe0c1,0x810603f,0x8105fc0,0x8105fbf,0x180fa040,0x180fa03f,0x180f9fc0,0x180f9fbf,0x8106041,0x81060c0,0x81060bf,0x8105fc1,0x180fa041,0x180fa0c0,0x180fa0bf,0x6fe03e,0x70203e,0x3101f40,0x30fdf40,0x180f9fc1,0x30fdf3f,0x3101f3f,0x3701fbe,0x36fdfbe,0x93060c1,0x192fa0c1,0x6fe0be,0x7020be,0x3101f41,0x30fdf41,0xb305f40,0xb105f3f,0x970603e,0xb705fbe,0x196fa03e,0x186f9fbe,0x1b2f9f40,0x1b6f9f3f,0x192fe042,0x9302042,0xb301fc2,0x1b2fdfc2,0x192fe140,0x9302140,0x970213f,0x196fe13f,
	15,0xfe040,0xfdfc0,0x101fc0,0x10203f,0xfe03f,0xfdfbf,0x101fbf,0x102041,0xfe041,0xfdfc1,0x101fc1,0x1020c0,0xfe0c0,0xfe0bf,0x1020bf,0x1020c1,0xfe0c1,0x8106040,0x8105fc0,0x810603f,0x8105fbf,0x180fa040,0x180f9fc0,0x8106041,0x8105fc1,0x81060c0,0x180fa03f,0x180f9fbf,0x180fa041,0x180f9fc1,0x180fa0c0,0x81060bf,0x91060c1,0x3101f40,0x30fdf40,0x30fdf3f,0x180fa0bf,0x190fa0c1,0x3101f3f,0x3101f41,0x30fdf41,0x186fe03e,0x70203e,0x3701fbe,0x1b6fdfbe,0x186fe0be,0x7020be,0x8302042,0x182fe042,0x1b2fdfc2,0xb301fc2,0xb105f40,0xb705f3f,0xb305f41,0x1b2f9f40,0x1b6f9f3f,0x192fe140,0x9302140,0x93020c2,0x192fe0c2,0x196fe13f,0x970213f,0xa70603e,
	11,0x10203f,0xfe040,0xfe03f,0xfdfbf,0x101fbf,0x101fc0,0xfdfc0,0xfe0c0,0x1020c0,0x1020bf,0xfe0bf,0x810603f,0x8106040,0x302041,0x2fe041,0x2fdfc1,0x301fc1,0x8105fc0,0x8105fbf,0x70203e,0x6fe03e,0x180fa03f,0x180fa040,0x2fe0c1,0x3020c1,0x81060c0,0x81060bf,0x701fbe,0x6fdfbe,0x180f9fbf,0x180f9fc0,0x180fa0c0,0x180fa0bf,0x6fe0be,0x7020be,0x8306041,0x8305fc1,0x3101f40,0x3101f3f,0x30fdf3f,0x30fdf40,0x182fa041,0x870603e,0x8705fbe,0x93060c1,0x182f9fc1,0x192fa0c1,0x186fa03e,0x186f9fbe,0x97060be,0x970213f,0x9302140,0x192fe140,0x196fe13f,0x196fa0be,0x32fdf41,0x3301f41,0xb305f40,0xb105f3f,0xb701f3e,0x36fdf3e,0x1b6f9f3f,0x1b2f9f40,
	11,0xfe040,0x10203f,0xfe03f,0xfdfc0,0x101fc0,0x101fbf,0xfdfbf,0xfe0c0,0x1020c0,0x1020bf,0xfe0bf,0x2fe041,0x302041,0x301fc1,0x2fdfc1,0x8106040,0x810603f,0x3020c1,0x2fe0c1,0x8105fc0,0x8105fbf,0x180fa040,0x180fa03f,0x81060c0,0x81060bf,0x70203e,0x180f9fc0,0x180f9fbf,0x6fe03e,0x180fa0c0,0x180fa0bf,0x8306041,0x701fbe,0x6fdfbe,0x6fe0be,0x7020be,0x8305fc1,0x182fa041,0x83060c1,0x182f9fc1,0x1b0fdf40,0x3101f40,0x3701f3f,0x1b6fdf3f,0x182fa0c1,0x190fe140,0x870603e,0x8705fbe,0x1102140,0x170213f,0x196fe13f,0x186fa03e,0x1b6f9fbe,0x87060be,0x3301f41,0x1b2fdf41,0xb305f40,0xb705f3f,0x196fa0be,0x192fe141,0x1302141,0x9306140,0x970613f,
	15,0xfe040,0xfe03f,0x10203f,0x101fc0,0xfdfc0,0xfdfbf,0x101fbf,0x1020c0,0xfe0c0,0xfe0bf,0x1020bf,0x102041,0xfe041,0xfdfc1,0x101fc1,0x1020c1,0xfe0c1,0x8106040,0x810603f,0x8105fc0,0x8105fbf,0x180fa040,0x180fa03f,0x81060c0,0x81060bf,0x8106041,0x180f9fc0,0x180f9fbf,0x180fa0c0,0x180fa0bf,0x180fa041,0x8105fc1,0x83060c1,0x70203e,0x6fe03e,0x6fdfbe,0x180f9fc1,0x182fa0c1,0x701fbe,0x7020be,0x6fe0be,0x1b0fdf40,0x3101f40,0x3701f3f,0x1b6fdf3f,0x1b0fdf41,0x3101f41,0x9102140,0x190fe140,0x196fe13f,0x970213f,0x870603e,0xb705fbe,0x97060be,0x196fa03e,0x1b6f9fbe,0x192fe042,0x9302042,0x9302141,0x192fe141,0x1b2fdfc2,0xb301fc2,0xb505f40,
	17,0xfe040,0xfe03f,0x10203f,0x101fc0,0xfdfc0,0xfe041,0x102041,0x1020c0,0xfe0c0,0xfdfbf,0x101fbf,0x1020bf,0xfe0bf,0xfdfc1,0x101fc1,0x1020c1,0xfe0c1,0x8106040,0x810603f,0x8105fc0,0x8106041,0x81060c0,0x180fa040,0x180fa03f,0x180f9fc0,0x8105fbf,0x81060bf,0x8105fc1,0x180fa041,0x180fa0c0,0x180f9fbf,0x81060c1,0x180fa0bf,0x180f9fc1,0x180fa0c1,0x186fe03e,0x70203e,0x3101f40,0x1b0fdf40,0x1b0fdf3f,0x3101f3f,0x3701fbe,0x1b6fdfbe,0x186fe0be,0x7020be,0x9102140,0x190fe140,0x182fe042,0x8302042,0x3101f41,0x1b0fdf41,0x1b2fdfc2,0xb301fc2,0x83020c2,0x182fe0c2,0x196fe13f,0x970213f,0x9302141,0x192fe141,0x970603e,0xb305f40,0xb105f3f,0xb705fbe,
	11,0x10203f,0x101fc0,0x101fbf,0xfe040,0xfe03f,0xfdfc0,0xfdfbf,0x105fbf,0x10603f,0x106040,0x105fc0,0x301fc1,0x302041,0x11020c0,0x11020bf,0x10fe0bf,0x10fe0c0,0x2fe041,0x2fdfc1,0x3101f40,0x3101f3f,0x701fbe,0x70203e,0x6fe03e,0x6fdfbe,0x30fdf3f,0x30fdf40,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x11060bf,0x11060c0,0x306041,0x305fc1,0x3105f40,0x3105f3f,0x705fbe,0x70603e,0x13020c1,0x12fe0c1,0x3301f41,0x32fdf41,0x17020be,0x16fe0be,0x190fa0bf,0x190fa0c0,0x192fa041,0x182f9fc1,0x3701f3e,0x36fdf3e,0x186f9fbe,0x196fa03e,0x1b6f9f3f,0x1b2f9f40,0x93060c1,0x97060be,0xb305f41,0xb705f3e,0xb709fbf,0x970a03f,0x930a040,0xb309fc0,
	9,0x101fc0,0x10203f,0xfe040,0xfdfc0,0x101fbf,0xfdfbf,0xfe03f,0x102041,0x101fc1,0x8105fc0,0x8106040,0x810603f,0x8105fbf,0xfdfc1,0xfe041,0x11020c0,0x11020bf,0x10fe0c0,0x10fe0bf,0x3101f40,0x30fdf40,0x3101f3f,0x30fdf3f,0x8105fc1,0x8106041,0x70203e,0x701fbe,0x6fdfbe,0x6fe03e,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x11020c1,0x91060c0,0x91060bf,0x12fe0c1,0x3101f41,0xb105f40,0xb105f3f,0x30fdf41,0x180f9fc1,0x182fa041,0x870603e,0x8705fbe,0x93060c1,0x17020be,0x16fe0be,0x190fa0bf,0x190fa0c0,0xb305f41,0x1b0f9f40,0x1b0f9f3f,0x36fdf3e,0xb701f3e,0x1b6f9fbe,0x196fa03e,0x97060be,0x192fa0c1,0x1b2f9f41,0x1b2fdfc2,0xb301fc2,0x11302042,
	11,0x101fc0,0xfe040,0xfdfc0,0x101fbf,0x10203f,0xfe03f,0xfdfbf,0x101fc1,0x102041,0xfe041,0xfdfc1,0x8105fc0,0x8106040,0x810603f,0x8105fbf,0x11020c0,0x10fe0c0,0x11020bf,0x10fe0bf,0x8106041,0x8105fc1,0x3101f40,0x30fdf40,0x3101f3f,0x30fdf3f,0x180f9fc0,0x180fa040,0x11020c1,0x10fe0c1,0x180fa03f,0x180f9fbf,0x91060c0,0x91060bf,0x3101f41,0x30fdf41,0x701fbe,0x70203e,0x6fe03e,0x6fdfbe,0xb105f40,0xb105f3f,0x180f9fc1,0x180fa041,0x93060c1,0xb305f41,0x190fa0c0,0x190fa0bf,0x870603e,0x8705fbe,0x97020be,0x16fe0be,0x1b0f9f40,0x1b0f9f3f,0x36fdf3e,0xb701f3e,0x192fa0c1,0x192fe042,0x9302042,0xb301fc2,0x1b2fdfc2,0x1b2f9f41,0x1b6f9fbe,0x196fa03e,
	11,0x101fc0,0xfe040,0xfdfc0,0x101fbf,0x10203f,0xfe03f,0xfdfbf,0x101fc1,0x102041,0xfe041,0xfdfc1,0x8105fc0,0x8106040,0x810603f,0x8105fbf,0x11020c0,0x10fe0c0,0x8106041,0x8105fc1,0x11020bf,0x10fe0bf,0x3101f40,0x30fdf40,0x11020c1,0x10fe0c1,0x180fa040,0x3101f3f,0x30fdf3f,0x180f9fc0,0x3101f41,0x30fdf41,0x91060c0,0x180fa03f,0x180f9fbf,0x180f9fc1,0x180fa041,0x91060bf,0xb105f40,0x91060c1,0xb105f3f,0x3701fbe,0x70203e,0x186fe03e,0x1b6fdfbe,0xb105f41,0x3301fc2,0x190fa0c0,0x190fa0bf,0x302042,0x182fe042,0x1b2fdfc2,0x1b0f9f40,0x1b6f9f3f,0x190fa0c1,0x870603e,0xb705fbe,0x97020be,0x196fe0be,0x1b2f9f41,0xb305fc2,0x8306042,0x93020c2,0x192fe0c2,
	9,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0xfdfbf,0xfdfc0,0x1020c0,0x1020bf,0x810603f,0x8106040,0x8105fc0,0x8105fbf,0xfe0bf,0xfe0c0,0x302041,0x301fc1,0x2fe041,0x2fdfc1,0x70203e,0x6fe03e,0x701fbe,0x6fdfbe,0x81060bf,0x81060c0,0x3101f40,0x3101f3f,0x30fdf3f,0x30fdf40,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x3020c1,0x8306041,0x8305fc1,0x12fe0c1,0x7020be,0x870603e,0x8705fbe,0x6fe0be,0x180fa0bf,0x190fa0c0,0xb105f40,0xb105f3f,0x93060c1,0x3301f41,0x32fdf41,0x182f9fc1,0x182fa041,0x97060be,0x186fa03e,0x186f9fbe,0x36fdf3e,0xb701f3e,0x1b6f9f3f,0x1b2f9f40,0xb305f41,0x192fa0c1,0x196fa0be,0x196fe13f,0x970213f,0x11302140,
	7,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0xfdfc0,0xfdfbf,0x302041,0x1020c0,0x8106040,0x810603f,0x1020bf,0xfe0c0,0x2fe041,0x301fc1,0x8105fc0,0x8105fbf,0xfe0bf,0x2fdfc1,0x3020c1,0x8306041,0x81060c0,0x81060bf,0x2fe0c1,0x8305fc1,0x3101f40,0x3101f3f,0x701fbe,0x70203e,0x6fe03e,0x180fa03f,0x180fa040,0x180f9fc0,0x30fdf40,0x30fdf3f,0x6fdfbe,0x180f9fbf,0x180fa0c0,0x182fa041,0x93060c1,0x870603e,0x7020be,0x6fe0be,0x180fa0bf,0x182f9fc1,0x32fdf41,0x3301f41,0xb105f40,0xb105f3f,0x8705fbe,0x97060be,0x192fa0c1,0xb305f41,0xb701f3e,0x36fdf3e,0x1b0f9f3f,0x1b2f9f40,0x1b6f9fbe,0x196fa03e,0x196fe13f,0x9302140,0x970213f,0x192fe140,
	11,0x101fc0,0xfe040,0xfdfc0,0x10203f,0x101fbf,0xfe03f,0xfdfbf,0x102041,0x101fc1,0xfe041,0xfdfc1,0x11020c0,0x8106040,0x8105fc0,0x10fe0c0,0x11020bf,0x810603f,0x8105fbf,0x10fe0bf,0x11020c1,0x8106041,0x8105fc1,0x10fe0c1,0x91060c0,0x91060bf,0x3101f40,0x30fdf40,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x30fdf3f,0x3101f3f,0x701fbe,0x70203e,0x6fe03e,0x6fdfbe,0x93060c1,0x3101f41,0x30fdf41,0x180f9fc1,0x180fa041,0x190fa0c0,0x190fa0bf,0xb105f40,0xb105f3f,0x8705fbe,0x870603e,0x17020be,0x16fe0be,0x192fa0c1,0xb305f41,0xb301fc2,0x9302042,0x192fe042,0x1b2fdfc2,0x1b2f9f40,0x1b0f9f3f,0x186f9fbe,0x196fa03e,0x97060be,0xb701f3e,0x1b6fdf3e,
	11,0x101fc0,0xfe040,0xfdfc0,0x10203f,0x101fbf,0xfe03f,0xfdfbf,0x102041,0x101fc1,0xfe041,0xfdfc1,0x11020c0,0x8106040,0x8105fc0,0x10fe0c0,0x11020bf,0x810603f,0x8105fbf,0x10fe0bf,0x11020c1,0x8106041,0x8105fc1,0x10fe0c1,0x91060c0,0x91060bf,0x3101f40,0x30fdf40,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x30fdf3f,0x3101f3f,0x3101f41,0x91060c1,0x180fa041,0x180f9fc1,0x30fdf41,0xb105f40,0x70203e,0x3701fbe,0x186fe03e,0x1b6fdfbe,0x190fa0c0,0x190fa0bf,0x190fa0c1,0xb105f3f,0xb105f41,0x3301fc2,0x302042,0x182fe042,0x1b2fdfc2,0x1b0f9f40,0x17020be,0x970603e,0xb705fbe,0x196fe0be,0x1b6f9f3f,0x1b2f9f41,0x13020c2,0x9306042,0xb305fc2,0x192fe0c2,
	11,0x10203f,0xfe040,0xfe03f,0x101fbf,0x101fc0,0xfdfc0,0xfdfbf,0x1020bf,0x1020c0,0xfe0c0,0xfe0bf,0x810603f,0x8106040,0x8105fc0,0x8105fbf,0x302041,0x2fe041,0x301fc1,0x2fdfc1,0x81060c0,0x81060bf,0x70203e,0x6fe03e,0x701fbe,0x6fdfbe,0x180fa03f,0x180fa040,0x3020c1,0x2fe0c1,0x180f9fc0,0x180f9fbf,0x8306041,0x8305fc1,0x7020be,0x6fe0be,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x870603e,0x8705fbe,0x180fa0bf,0x180fa0c0,0x93060c1,0x97060be,0x182fa041,0x182f9fc1,0xb105f40,0xb105f3f,0xb301f41,0x32fdf41,0x186fa03e,0x186f9fbe,0x36fdf3e,0xb701f3e,0x192fa0c1,0x192fe140,0x9302140,0x970213f,0x196fe13f,0x196fa0be,0x1b6f9f3f,0x1b2f9f40,
	11,0x10203f,0xfe040,0xfe03f,0x101fc0,0x101fbf,0xfdfc0,0xfdfbf,0x1020c0,0x1020bf,0xfe0c0,0xfe0bf,0x302041,0x8106040,0x810603f,0x2fe041,0x301fc1,0x8105fc0,0x8105fbf,0x2fdfc1,0x3020c1,0x81060c0,0x81060bf,0x2fe0c1,0x8306041,0x8305fc1,0x70203e,0x6fe03e,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x6fdfbe,0x701fbe,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x93060c1,0x7020be,0x6fe0be,0x180fa0bf,0x180fa0c0,0x182fa041,0x182f9fc1,0x870603e,0x8705fbe,0xb105f3f,0xb105f40,0x3301f41,0x32fdf41,0x192fa0c1,0x97060be,0x970213f,0x9302140,0x192fe140,0x196fe13f,0x196fa03e,0x186f9fbe,0x1b0f9f3f,0x1b2f9f40,0xb305f41,0xb701f3e,0x1b6fdf3e,
	15,0xfe040,0x10203f,0x101fc0,0xfdfc0,0xfe03f,0x101fbf,0xfdfbf,0x102041,0x1020c0,0xfe0c0,0xfe041,0x101fc1,0xfdfc1,0x1020bf,0xfe0bf,0x8106040,0x810603f,0x8105fc0,0x8105fbf,0x1020c1,0xfe0c1,0x8106041,0x81060c0,0x81060bf,0x8105fc1,0x180fa040,0x180f9fc0,0x180fa03f,0x180f9fbf,0x93060c1,0x70203e,0x6fe03e,0x3101f40,0x1b0fdf40,0x3101f3f,0x3701fbe,0x6fdfbe,0x1b6fdf3f,0x180fa041,0x180fa0c0,0x180fa0bf,0x180f9fc1,0x1b0fdf41,0x3101f41,0x7020be,0x6fe0be,0x870603e,0x8705fbe,0xb105f40,0xb705f3f,0x192fa0c1,0x192fe140,0x9302140,0x970213f,0x97060be,0x9302042,0x192fe042,0xb301fc2,0xb305f41,0x1b2fdfc2,0x196fe13f,0x196fa03e,0x1b6f9fbe,
	14,0xfe040,0x101fc0,0xfdfc0,0x10203f,0xfe03f,0x101fbf,0xfdfbf,0x102041,0xfe041,0x101fc1,0xfdfc1,0x1020c0,0xfe0c0,0x1020bf,0x8106040,0xfe0bf,0x8105fc0,0x1020c1,0xfe0c1,0x810603f,0x8105fbf,0x8106041,0x8105fc1,0x81060c0,0x81060bf,0x91060c1,0x180fa040,0x180f9fc0,0x180fa03f,0x180f9fbf,0x180fa041,0x180f9fc1,0x1b0fdf40,0x3101f40,0x3101f3f,0x1b0fdf3f,0x180fa0c0,0x190fa0bf,0x186fe03e,0x70203e,0x3701fbe,0x3101f41,0x1b0fdf41,0x1b6fdfbe,0x190fa0c1,0x182fe042,0x302042,0xb105f40,0xb105f3f,0x3301fc2,0x1b2fdfc2,0x7020be,0x196fe0be,0x970603e,0xb705fbe,0xb105f41,0x9302140,0x192fe140,0x13020c2,0x192fe0c2,0x9306042,0xb305fc2,0x1170213f,
	11,0x10203f,0xfe040,0xfe03f,0x101fbf,0x101fc0,0xfdfc0,0xfdfbf,0x1020bf,0x1020c0,0xfe0c0,0xfe0bf,0x810603f,0x8106040,0x8105fc0,0x8105fbf,0x302041,0x2fe041,0x81060c0,0x81060bf,0x301fc1,0x2fdfc1,0x70203e,0x6fe03e,0x3020c1,0x2fe0c1,0x180fa040,0x701fbe,0x6fdfbe,0x180fa03f,0x7020be,0x6fe0be,0x8306041,0x180f9fc0,0x180f9fbf,0x180fa0bf,0x180fa0c0,0x8305fc1,0x870603e,0x83060c1,0x8705fbe,0x3701f3f,0x3101f40,0x1b0fdf40,0x1b6fdf3f,0x87060be,0x170213f,0x182fa041,0x182f9fc1,0x1102140,0x190fe140,0x196fe13f,0x186fa03e,0x1b6f9fbe,0x182fa0c1,0xb105f40,0xb705f3f,0xb301f41,0x1b2fdf41,0x196fa0be,0x970613f,0x9106140,0x9302141,0x192fe141,
	11,0x10203f,0xfe040,0xfe03f,0x101fc0,0x101fbf,0xfdfc0,0xfdfbf,0x1020c0,0x1020bf,0xfe0c0,0xfe0bf,0x302041,0x8106040,0x810603f,0x2fe041,0x301fc1,0x8105fc0,0x8105fbf,0x2fdfc1,0x3020c1,0x81060c0,0x81060bf,0x2fe0c1,0x8306041,0x8305fc1,0x70203e,0x6fe03e,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x6fdfbe,0x701fbe,0x7020be,0x83060c1,0x180fa0c0,0x180fa0bf,0x6fe0be,0x870603e,0x3101f40,0x3701f3f,0x1b0fdf40,0x1b6fdf3f,0x182fa041,0x182f9fc1,0x182fa0c1,0x8705fbe,0x87060be,0x170213f,0x1102140,0x190fe140,0x196fe13f,0x186fa03e,0x3301f41,0xb305f40,0xb705f3f,0x1b2fdf41,0x1b6f9fbe,0x196fa0be,0x1302141,0x9306140,0x970613f,0x192fe141,
	14,0xfe040,0x10203f,0xfe03f,0x101fc0,0xfdfc0,0x101fbf,0xfdfbf,0x1020c0,0xfe0c0,0x1020bf,0xfe0bf,0x102041,0xfe041,0x101fc1,0x8106040,0xfdfc1,0x810603f,0x1020c1,0xfe0c1,0x8105fc0,0x8105fbf,0x81060c0,0x81060bf,0x8106041,0x8105fc1,0x83060c1,0x180fa040,0x180fa03f,0x180f9fc0,0x180f9fbf,0x180fa0c0,0x180fa0bf,0x186fe03e,0x70203e,0x701fbe,0x186fdfbe,0x180fa041,0x182f9fc1,0x1b0fdf40,0x3101f40,0x3701f3f,0x7020be,0x186fe0be,0x1b6fdf3f,0x182fa0c1,0x190fe140,0x1102140,0x870603e,0x8705fbe,0x170213f,0x196fe13f,0x3101f41,0x1b2fdf41,0xb305f40,0xb705f3f,0x87060be,0x9302042,0x192fe042,0x1302141,0x192fe141,0x9306140,0x970613f,0x13301fc2,
	17,0xfe040,0x10203f,0x101fc0,0xfdfc0,0xfe03f,0x1020c0,0x102041,0xfe041,0xfe0c0,0x101fbf,0xfdfbf,0x1020bf,0xfe0bf,0x101fc1,0xfdfc1,0x1020c1,0xfe0c1,0x8106040,0x810603f,0x8105fc0,0x8106041,0x81060c0,0x8105fbf,0x81060bf,0x8105fc1,0x81060c1,0x180fa040,0x180f9fc0,0x180fa03f,0x180fa0c0,0x180fa041,0x180f9fbf,0x180fa0bf,0x180f9fc1,0x180fa0c1,0x70203e,0x186fe03e,0x3101f40,0x1b0fdf40,0x3101f3f,0x3701fbe,0x186fdfbe,0x1b6fdf3f,0x3101f41,0x1b0fdf41,0x8302042,0x182fe042,0x9102140,0x7020be,0x186fe0be,0x190fe140,0x970213f,0x196fe13f,0x9102141,0xb301fc2,0x1b2fdfc2,0x93020c2,0x182fe0c2,0x192fe141,0x970603e,0xb305f40,0xb105f3f,0xb705fbe,
	11,0x10203f,0x101fc0,0x101fbf,0xfdfbf,0xfe03f,0xfe040,0xfdfc0,0x105fc0,0x106040,0x10603f,0x105fbf,0x11020bf,0x11020c0,0x302041,0x301fc1,0x2fdfc1,0x2fe041,0x10fe0c0,0x10fe0bf,0x70203e,0x701fbe,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x6fdfbe,0x6fe03e,0x11060bf,0x11060c0,0x306041,0x305fc1,0x3105f40,0x3105f3f,0x705fbe,0x70603e,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x13020c1,0x12fe0c1,0x17020be,0x16fe0be,0x3301f41,0x32fdf41,0x93060c1,0x3701f3e,0x36fdf3e,0x97060be,0xb305f41,0x182f9fc1,0x182fa041,0x192fa0c0,0x190fa0bf,0x196fa03e,0x186f9fbe,0xb705f3e,0xb709fbf,0xb309fc0,0x930a040,0x970a03f,0x1b6f9f3f,0x1b2f9f40,
	11,0x101fc0,0x10203f,0x101fbf,0xfdfc0,0xfe040,0xfe03f,0xfdfbf,0x105fc0,0x106040,0x10603f,0x105fbf,0x301fc1,0x302041,0x2fe041,0x2fdfc1,0x11020c0,0x11020bf,0x10fe0c0,0x10fe0bf,0x306041,0x305fc1,0x3101f40,0x3101f3f,0x30fdf40,0x30fdf3f,0x701fbe,0x70203e,0x11060c0,0x11060bf,0x6fe03e,0x6fdfbe,0x13020c1,0x12fe0c1,0x3105f40,0x3105f3f,0x180f9fc0,0x180fa040,0x180fa03f,0x180f9fbf,0x3301f41,0x32fdf41,0x705fbe,0x70603e,0x93060c1,0xb305f41,0x17020be,0x16fe0be,0x182fa041,0x182f9fc1,0x192fa0c0,0x190fa0bf,0x3701f3e,0x36fdf3e,0x1b0f9f3f,0x1b2f9f40,0x97060be,0x970a03f,0x930a040,0xb309fc0,0xb709fbf,0xb705f3e,0x1b6f9fbe,0x196fa03e,
	15,0x101fc0,0x101fbf,0x10203f,0xfe040,0xfdfc0,0xfdfbf,0xfe03f,0x102041,0x101fc1,0x105fc0,0x106040,0x10603f,0x105fbf,0xfdfc1,0xfe041,0x11020c0,0x106041,0x105fc1,0x11020bf,0x10fe0c0,0x10fe0bf,0x3101f40,0x3101f3f,0x30fdf40,0x30fdf3f,0x11020c1,0x11060c0,0x11060bf,0x10fe0c1,0x3101f41,0x3105f40,0x3105f3f,0x701fbe,0x70203e,0x180fa040,0x180f9fc0,0x30fdf41,0x180f9fbf,0x180fa03f,0x186fe03e,0x186fdfbe,0x93060c1,0xb305f41,0x705fbe,0x70603e,0x180fa041,0x180f9fc1,0x192fa0c0,0x190fa0bf,0x97020be,0x196fe0be,0xb701f3e,0x36fdf3e,0x1b2f9f40,0x1b6f9f3f,0xb301fc2,0x9302042,0x192fe042,0x1b2fdfc2,0xb309fc0,0x930a040,0x970a03f,0xb709fbf,
	15,0x101fc0,0xfdfc0,0xfe040,0x10203f,0x101fbf,0xfdfbf,0xfe03f,0x102041,0x101fc1,0xfdfc1,0xfe041,0x106040,0x105fc0,0x105fbf,0x10603f,0x106041,0x105fc1,0x11020c0,0x10fe0c0,0x11020bf,0x10fe0bf,0x3101f40,0x30fdf40,0x11020c1,0x10fe0c1,0x11060c0,0x3101f3f,0x30fdf3f,0x3101f41,0x30fdf41,0x3105f40,0x11060bf,0x91060c1,0x180fa040,0x180f9fc0,0x180f9fbf,0x3105f3f,0xb105f41,0x180fa03f,0x180fa041,0x180f9fc1,0x3701fbe,0x70203e,0x186fe03e,0x1b6fdfbe,0x3705fbe,0x70603e,0x1302042,0x3301fc2,0x1b2fdfc2,0x192fe042,0x190fa0c0,0x196fa0bf,0x192fa0c1,0x1b2f9f40,0x1b6f9f3f,0xb309fc0,0x930a040,0x9306042,0xb305fc2,0xb709fbf,0x970a03f,0x117020be,
	11,0x10203f,0x101fc0,0x101fbf,0xfe03f,0xfe040,0xfdfc0,0xfdfbf,0x10603f,0x106040,0x105fc0,0x105fbf,0x11020bf,0x11020c0,0x10fe0c0,0x10fe0bf,0x302041,0x301fc1,0x2fe041,0x2fdfc1,0x11060c0,0x11060bf,0x70203e,0x701fbe,0x6fe03e,0x6fdfbe,0x3101f3f,0x3101f40,0x306041,0x305fc1,0x30fdf40,0x30fdf3f,0x13020c1,0x12fe0c1,0x70603e,0x705fbe,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x17020be,0x16fe0be,0x3105f3f,0x3105f40,0x93060c1,0x97060be,0x3301f41,0x32fdf41,0x190fa0c0,0x190fa0bf,0x192fa041,0x182f9fc1,0x3701f3e,0x36fdf3e,0x186f9fbe,0x196fa03e,0xb305f41,0xb309fc0,0x930a040,0x970a03f,0xb709fbf,0xb705f3e,0x1b6f9f3f,0x1b2f9f40,
	11,0x10203f,0x101fc0,0x101fbf,0xfe040,0xfe03f,0xfdfc0,0xfdfbf,0x106040,0x10603f,0x105fc0,0x105fbf,0x302041,0x11020c0,0x11020bf,0x301fc1,0x2fe041,0x10fe0c0,0x10fe0bf,0x2fdfc1,0x306041,0x11060c0,0x11060bf,0x305fc1,0x13020c1,0x12fe0c1,0x70203e,0x701fbe,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x6fdfbe,0x6fe03e,0x180fa03f,0x180fa040,0x180f9fc0,0x180f9fbf,0x93060c1,0x70603e,0x705fbe,0x3105f3f,0x3105f40,0x3301f41,0x32fdf41,0x17020be,0x16fe0be,0x190fa0bf,0x190fa0c0,0x182fa041,0x182f9fc1,0xb305f41,0x97060be,0x970a03f,0x930a040,0xb309fc0,0xb709fbf,0xb701f3e,0x36fdf3e,0x1b0f9f3f,0x1b2f9f40,0x192fa0c1,0x196fa03e,0x1b6f9fbe,
	15,0x101fc0,0x10203f,0xfe040,0xfdfc0,0x101fbf,0xfe03f,0xfdfbf,0x102041,0x106040,0x105fc0,0x101fc1,0xfe041,0xfdfc1,0x10603f,0x105fbf,0x11020c0,0x11020bf,0x10fe0c0,0x10fe0bf,0x106041,0x105fc1,0x11020c1,0x11060c0,0x11060bf,0x10fe0c1,0x3101f40,0x30fdf40,0x3101f3f,0x30fdf3f,0x93060c1,0x70203e,0x701fbe,0x180fa040,0x1b0f9fc0,0x180fa03f,0x186fe03e,0x6fdfbe,0x1b6f9fbf,0x3101f41,0x3105f40,0x3105f3f,0x30fdf41,0x1b0f9fc1,0x180fa041,0x70603e,0x705fbe,0x17020be,0x16fe0be,0x190fa0c0,0x196fa0bf,0xb305f41,0xb309fc0,0x930a040,0x970a03f,0x97060be,0x9302042,0xb301fc2,0x192fe042,0x192fa0c1,0x1b2fdfc2,0xb709fbf,0xb701f3e,0x1b6fdf3e,
	14,0x101fc0,0xfe040,0xfdfc0,0x10203f,0x101fbf,0xfe03f,0xfdfbf,0x102041,0x101fc1,0xfe041,0xfdfc1,0x106040,0x105fc0,0x10603f,0x11020c0,0x105fbf,0x10fe0c0,0x106041,0x105fc1,0x11020bf,0x10fe0bf,0x11020c1,0x10fe0c1,0x11060c0,0x11060bf,0x91060c1,0x3101f40,0x30fdf40,0x3101f3f,0x30fdf3f,0x3101f41,0x30fdf41,0x1b0f9fc0,0x180fa040,0x180fa03f,0x1b0f9fbf,0x3105f40,0xb105f3f,0x3701fbe,0x70203e,0x186fe03e,0x180fa041,0x1b0f9fc1,0x1b6fdfbe,0xb105f41,0x3301fc2,0x302042,0x190fa0c0,0x190fa0bf,0x182fe042,0x1b2fdfc2,0x70603e,0xb705fbe,0x97020be,0x196fe0be,0x190fa0c1,0x930a040,0xb309fc0,0x8306042,0xb305fc2,0x93020c2,0x192fe0c2,0xa70a03f,
	15,0x10203f,0x101fbf,0x101fc0,0xfe040,0xfe03f,0xfdfbf,0xfdfc0,0x1020c0,0x1020bf,0x10603f,0x106040,0x105fc0,0x105fbf,0xfe0bf,0xfe0c0,0x302041,0x1060c0,0x1060bf,0x301fc1,0x2fe041,0x2fdfc1,0x70203e,0x701fbe,0x6fe03e,0x6fdfbe,0x3020c1,0x306041,0x305fc1,0x2fe0c1,0x7020be,0x70603e,0x705fbe,0x3101f3f,0x3101f40,0x180fa040,0x180fa03f,0x6fe0be,0x180f9fbf,0x180f9fc0,0x1b0fdf40,0x1b0fdf3f,0x93060c1,0x97060be,0x3105f3f,0x3105f40,0x180fa0c0,0x180fa0bf,0x192fa041,0x182f9fc1,0xb301f41,0x1b2fdf41,0xb701f3e,0x36fdf3e,0x196fa03e,0x1b6f9fbe,0x970213f,0x9302140,0x192fe140,0x196fe13f,0x970a03f,0x930a040,0xb309fc0,0xb709fbf,
	15,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0xfdfc0,0xfdfbf,0x1020c0,0x106040,0x10603f,0x1020bf,0xfe0c0,0xfe0bf,0x105fc0,0x105fbf,0x302041,0x301fc1,0x2fe041,0x2fdfc1,0x1060c0,0x1060bf,0x3020c1,0x306041,0x305fc1,0x2fe0c1,0x70203e,0x6fe03e,0x701fbe,0x6fdfbe,0x93060c1,0x3101f40,0x3101f3f,0x180fa040,0x186fa03f,0x180f9fc0,0x1b0fdf40,0x30fdf3f,0x1b6f9fbf,0x7020be,0x70603e,0x705fbe,0x6fe0be,0x186fa0bf,0x180fa0c0,0x3105f40,0x3105f3f,0x3301f41,0x32fdf41,0x182fa041,0x1b2f9fc1,0x97060be,0x970a03f,0x930a040,0xb309fc0,0xb305f41,0x9302140,0x970213f,0x192fe140,0x192fa0c1,0x196fe13f,0xb709fbf,0xb701f3e,0x1b6fdf3e,
	16,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0xfdfc0,0xfdfbf,0x102041,0x1020c0,0x106040,0x10603f,0x1020bf,0xfe0c0,0xfe041,0x101fc1,0x105fc0,0x105fbf,0xfe0bf,0xfdfc1,0x1020c1,0x106041,0x1060c0,0x1060bf,0xfe0c1,0x105fc1,0x93060c1,0x70203e,0x3101f40,0x180fa040,0x180fa03f,0x186fe03e,0x701fbe,0x3701f3f,0x30fdf40,0x1b0f9fc0,0x180f9fbf,0x186fdfbe,0x1b6fdf3f,0x3101f41,0x3105f40,0xb105f3f,0x70603e,0x7020be,0x6fe0be,0x180fa0c0,0x180fa041,0x182f9fc1,0x1b2fdf41,0xb705fbe,0x186fa0bf,0x192fa0c1,0x97060be,0xb305f41,0x9302042,0x192fe042,0x13301fc2,0x930a040,0x970a03f,0xb509fc0,0x9302140,0x970213f,0x192fe140,0x196fe13f,
	15,0x101fc0,0xfe040,0xfdfc0,0x10203f,0x101fbf,0xfe03f,0x102041,0x101fc1,0xfdfbf,0xfe041,0x1020c0,0x106040,0xfdfc1,0x105fc0,0xfe0c0,0x1020bf,0x10603f,0x105fbf,0xfe0bf,0x1020c1,0x106041,0x105fc1,0xfe0c1,0x1060c0,0x11060bf,0x91060c1,0x3101f40,0x180fa040,0x180f9fc0,0x1b0fdf40,0x3101f3f,0x30fdf3f,0x180fa03f,0x1b0f9fbf,0x180fa041,0x180f9fc1,0x3101f41,0x1b0fdf41,0x70203e,0x3701fbe,0x186fe03e,0x1b6fdfbe,0x3105f40,0xb105f3f,0x180fa0c0,0x190fa0bf,0x192fa0c1,0x302042,0x3301fc2,0xb305f41,0x182fe042,0x1b2fdfc2,0x17020be,0x170603e,0xb705fbe,0x196fe0be,0x9502140,0x194fe140,0x193020c2,0xa306042,0x930a040,0xb309fc0,0xa70a03f,
	15,0x10203f,0xfe03f,0xfe040,0x101fc0,0x101fbf,0xfdfbf,0xfdfc0,0x1020c0,0x1020bf,0xfe0bf,0xfe0c0,0x106040,0x10603f,0x105fbf,0x105fc0,0x1060c0,0x1060bf,0x302041,0x2fe041,0x301fc1,0x2fdfc1,0x70203e,0x6fe03e,0x3020c1,0x2fe0c1,0x306041,0x701fbe,0x6fdfbe,0x7020be,0x6fe0be,0x70603e,0x305fc1,0x83060c1,0x180fa040,0x180fa03f,0x180f9fbf,0x705fbe,0x87060be,0x180f9fc0,0x180fa0c0,0x180fa0bf,0x3701f3f,0x3101f40,0x1b0fdf40,0x1b6fdf3f,0x3705f3f,0x3105f40,0x1302140,0x170213f,0x196fe13f,0x192fe140,0x182fa041,0x1b2f9fc1,0x192fa0c1,0x196fa03e,0x1b6f9fbe,0x970a03f,0x930a040,0x9306140,0x970613f,0xb709fbf,0xb309fc0,0x13301f41,
	14,0x10203f,0xfe040,0xfe03f,0x101fc0,0x101fbf,0xfdfc0,0xfdfbf,0x1020c0,0x1020bf,0xfe0c0,0xfe0bf,0x106040,0x10603f,0x105fc0,0x302041,0x105fbf,0x2fe041,0x1060c0,0x1060bf,0x301fc1,0x2fdfc1,0x3020c1,0x2fe0c1,0x306041,0x305fc1,0x83060c1,0x70203e,0x6fe03e,0x701fbe,0x6fdfbe,0x7020be,0x6fe0be,0x186fa03f,0x180fa040,0x180f9fc0,0x186f9fbf,0x70603e,0x8705fbe,0x3701f3f,0x3101f40,0x1b0fdf40,0x180fa0c0,0x186fa0bf,0x1b6fdf3f,0x87060be,0x170213f,0x1102140,0x182fa041,0x182f9fc1,0x190fe140,0x196fe13f,0x3105f40,0xb705f3f,0xb301f41,0x1b2fdf41,0x182fa0c1,0x930a040,0x970a03f,0x9106140,0x970613f,0x9302141,0x192fe141,0xb509fc0,
	15,0x10203f,0xfe040,0xfe03f,0x101fc0,0x101fbf,0xfdfc0,0x1020c0,0x1020bf,0xfdfbf,0xfe0c0,0x102041,0x106040,0xfe0bf,0x10603f,0xfe041,0x101fc1,0x105fc0,0x105fbf,0xfdfc1,0x1020c1,0x1060c0,0x1060bf,0xfe0c1,0x106041,0x305fc1,0x83060c1,0x70203e,0x180fa040,0x180fa03f,0x186fe03e,0x701fbe,0x6fdfbe,0x180f9fc0,0x186f9fbf,0x180fa0c0,0x180fa0bf,0x7020be,0x186fe0be,0x3101f40,0x3701f3f,0x1b0fdf40,0x1b6fdf3f,0x70603e,0x8705fbe,0x180fa041,0x182f9fc1,0x192fa0c1,0x1102140,0x170213f,0x97060be,0x190fe140,0x196fe13f,0x3301f41,0x3305f40,0xb705f3f,0x1b2fdf41,0xa302042,0x1a2fe042,0x19302141,0x9506140,0x930a040,0x970a03f,0xb509fc0,
	17,0xfe040,0x10203f,0x101fc0,0xfdfc0,0xfe03f,0x1020c0,0x102041,0x101fbf,0xfe041,0xfe0c0,0x106040,0xfdfbf,0x1020bf,0x101fc1,0xfdfc1,0xfe0bf,0x1020c1,0x10603f,0x105fc0,0xfe0c1,0x1060c0,0x106041,0x8105fbf,0x81060bf,0x8105fc1,0x81060c1,0x180fa040,0x180f9fc0,0x180fa03f,0x180fa0c0,0x180fa041,0x180f9fbf,0x70203e,0x186fe03e,0x3101f40,0x1b0fdf40,0x180f9fc1,0x180fa0bf,0x701fbe,0x3701f3f,0x1b0fdf3f,0x1b6fdfbe,0x7020be,0x186fe0be,0x192fa0c1,0x9102140,0x190fe140,0x8302042,0x3101f41,0x1b0fdf41,0x182fe042,0xb301fc2,0xb305f40,0x870603e,0x970213f,0x196fe13f,0x11102141,0x113020c2,0x1b2fdfc2,0xb105f3f,0xb705fbe,0x97060be,0xa50a040,
	11,0x10203f,0x101fc0,0x101fbf,0xfdfbf,0xfe03f,0xfe040,0xfdfc0,0x105fc0,0x106040,0x10603f,0x105fbf,0x11020bf,0x11020c0,0x302041,0x301fc1,0x2fdfc1,0x2fe041,0x10fe0c0,0x10fe0bf,0x70203e,0x701fbe,0x3101f3f,0x3101f40,0x305fc1,0x306041,0x11060c0,0x11060bf,0x6fe03e,0x6fdfbe,0x30fdf3f,0x30fdf40,0x3105f40,0x3105f3f,0x705fbe,0x70603e,0x13020c1,0x12fe0c1,0x180fa040,0x180fa03f,0x180f9fbf,0x180f9fc0,0x3301f41,0x17020be,0x16fe0be,0x93060c1,0x32fdf41,0xb305f41,0x3701f3e,0x36fdf3e,0x97060be,0x970a03f,0x930a040,0xb309fc0,0xb709fbf,0xb705f3e,0x182f9fc1,0x182fa041,0x192fa0c0,0x190fa0bf,0x196fa03e,0x186f9fbe,0x1b6f9f3f,0x1b2f9f40,
	11,0x101fc0,0x10203f,0x101fbf,0xfdfc0,0xfe040,0xfe03f,0xfdfbf,0x105fc0,0x106040,0x10603f,0x105fbf,0x301fc1,0x302041,0x2fe041,0x2fdfc1,0x11020c0,0x11020bf,0x306041,0x305fc1,0x10fe0c0,0x10fe0bf,0x3101f40,0x3101f3f,0x11060c0,0x11060bf,0x70203e,0x30fdf40,0x30fdf3f,0x701fbe,0x3105f40,0x3105f3f,0x13020c1,0x6fe03e,0x6fdfbe,0x705fbe,0x70603e,0x12fe0c1,0x3301f41,0x13060c1,0x32fdf41,0x1b0f9fc0,0x180fa040,0x186fa03f,0x1b6f9fbf,0x3305f41,0xb109fc0,0x17020be,0x16fe0be,0x810a040,0x870a03f,0xb709fbf,0x3701f3e,0x1b6fdf3e,0x17060be,0x182fa041,0x1b2f9fc1,0x192fa0c0,0x196fa0bf,0xb705f3e,0xb309fc1,0x830a041,0x930a0c0,0x970a0bf,
	15,0x101fc0,0x101fbf,0x10203f,0xfe040,0xfdfc0,0xfdfbf,0xfe03f,0x106040,0x105fc0,0x105fbf,0x10603f,0x102041,0x101fc1,0xfdfc1,0xfe041,0x106041,0x105fc1,0x11020c0,0x11020bf,0x10fe0c0,0x10fe0bf,0x3101f40,0x3101f3f,0x11060c0,0x11060bf,0x11020c1,0x30fdf40,0x30fdf3f,0x3105f40,0x3105f3f,0x3101f41,0x10fe0c1,0x13060c1,0x70203e,0x701fbe,0x6fdfbe,0x30fdf41,0x3305f41,0x6fe03e,0x70603e,0x705fbe,0x1b0f9fc0,0x180fa040,0x186fa03f,0x1b6f9fbf,0x1b0f9fc1,0x180fa041,0x910a040,0xb109fc0,0xb709fbf,0x970a03f,0x17020be,0x196fe0be,0x97060be,0xb701f3e,0x1b6fdf3e,0xb301fc2,0x9302042,0x930a041,0xb309fc1,0x1b2fdfc2,0x192fe042,0x194fa0c0,
	17,0x101fc0,0x101fbf,0x10203f,0xfe040,0xfdfc0,0x101fc1,0x102041,0x106040,0x105fc0,0xfdfbf,0xfe03f,0x10603f,0x105fbf,0xfdfc1,0xfe041,0x106041,0x105fc1,0x11020c0,0x11020bf,0x10fe0c0,0x11020c1,0x11060c0,0x3101f40,0x3101f3f,0x30fdf40,0x10fe0bf,0x11060bf,0x10fe0c1,0x3101f41,0x3105f40,0x30fdf3f,0x11060c1,0x3105f3f,0x30fdf41,0x3105f41,0x3701fbe,0x70203e,0x180fa040,0x1b0f9fc0,0x1b0f9fbf,0x180fa03f,0x186fe03e,0x1b6fdfbe,0x3705fbe,0x70603e,0x910a040,0xb109fc0,0x3301fc2,0x1302042,0x180fa041,0x1b0f9fc1,0x1b2fdfc2,0x192fe042,0x1306042,0x3305fc2,0xb709fbf,0x970a03f,0x930a041,0xb309fc1,0x97020be,0x192fa0c0,0x190fa0bf,0x196fe0be,
	11,0x10203f,0x101fc0,0x101fbf,0xfe03f,0xfe040,0xfdfc0,0xfdfbf,0x10603f,0x106040,0x105fc0,0x105fbf,0x11020bf,0x11020c0,0x10fe0c0,0x10fe0bf,0x302041,0x301fc1,0x11060c0,0x11060bf,0x2fe041,0x2fdfc1,0x70203e,0x701fbe,0x306041,0x305fc1,0x3101f40,0x6fe03e,0x6fdfbe,0x3101f3f,0x70603e,0x705fbe,0x13020c1,0x30fdf40,0x30fdf3f,0x3105f3f,0x3105f40,0x12fe0c1,0x17020be,0x13060c1,0x16fe0be,0x186fa03f,0x180fa040,0x1b0f9fc0,0x1b6f9fbf,0x17060be,0x870a03f,0x3301f41,0x32fdf41,0x810a040,0xb109fc0,0xb709fbf,0x3701f3e,0x1b6fdf3e,0x3305f41,0x190fa0c0,0x196fa0bf,0x192fa041,0x1b2f9fc1,0xb705f3e,0x970a0bf,0x910a0c0,0x930a041,0xb309fc1,
	11,0x10203f,0x101fc0,0x101fbf,0xfe040,0xfe03f,0xfdfc0,0xfdfbf,0x106040,0x10603f,0x105fc0,0x105fbf,0x302041,0x11020c0,0x11020bf,0x301fc1,0x2fe041,0x10fe0c0,0x10fe0bf,0x2fdfc1,0x306041,0x11060c0,0x11060bf,0x305fc1,0x13020c1,0x12fe0c1,0x70203e,0x701fbe,0x3101f3f,0x3101f40,0x30fdf40,0x30fdf3f,0x6fdfbe,0x6fe03e,0x70603e,0x13060c1,0x3105f40,0x3105f3f,0x705fbe,0x17020be,0x180fa040,0x186fa03f,0x1b0f9fc0,0x1b6f9fbf,0x3301f41,0x32fdf41,0x3305f41,0x16fe0be,0x17060be,0x870a03f,0x810a040,0xb109fc0,0xb709fbf,0x3701f3e,0x182fa041,0x192fa0c0,0x196fa0bf,0x1b2f9fc1,0x1b6fdf3e,0xb705f3e,0x830a041,0x930a0c0,0x970a0bf,0xb309fc1,
	14,0x101fc0,0x10203f,0x101fbf,0xfe040,0xfdfc0,0xfe03f,0xfdfbf,0x106040,0x105fc0,0x10603f,0x105fbf,0x102041,0x101fc1,0xfe041,0x11020c0,0xfdfc1,0x11020bf,0x106041,0x105fc1,0x10fe0c0,0x10fe0bf,0x11060c0,0x11060bf,0x11020c1,0x10fe0c1,0x13060c1,0x3101f40,0x3101f3f,0x30fdf40,0x30fdf3f,0x3105f40,0x3105f3f,0x3701fbe,0x70203e,0x6fe03e,0x36fdfbe,0x3101f41,0x32fdf41,0x1b0f9fc0,0x180fa040,0x186fa03f,0x70603e,0x3705fbe,0x1b6f9fbf,0x3305f41,0xb109fc0,0x810a040,0x17020be,0x16fe0be,0x870a03f,0xb709fbf,0x180fa041,0x1b2f9fc1,0x192fa0c0,0x196fa0bf,0x17060be,0x9302042,0xb301fc2,0x830a041,0xb309fc1,0x930a0c0,0x970a0bf,0x1a2fe042,
	17,0x101fc0,0x10203f,0xfe040,0xfdfc0,0x101fbf,0x106040,0x102041,0x101fc1,0x105fc0,0xfe03f,0xfdfbf,0x10603f,0x105fbf,0xfe041,0xfdfc1,0x106041,0x105fc1,0x11020c0,0x11020bf,0x10fe0c0,0x11020c1,0x11060c0,0x10fe0bf,0x11060bf,0x10fe0c1,0x11060c1,0x3101f40,0x30fdf40,0x3101f3f,0x3105f40,0x3101f41,0x30fdf3f,0x3105f3f,0x30fdf41,0x3105f41,0x70203e,0x3701fbe,0x180fa040,0x1b0f9fc0,0x180fa03f,0x186fe03e,0x36fdfbe,0x1b6f9fbf,0x180fa041,0x1b0f9fc1,0x1302042,0x3301fc2,0x910a040,0x70603e,0x3705fbe,0xb109fc0,0x970a03f,0xb709fbf,0x910a041,0x192fe042,0x1b2fdfc2,0x9306042,0x3305fc2,0xb309fc1,0x97020be,0x192fa0c0,0x190fa0bf,0x196fe0be,
	15,0x10203f,0x101fbf,0x101fc0,0xfe040,0xfe03f,0xfdfbf,0xfdfc0,0x106040,0x10603f,0x105fbf,0x105fc0,0x1020c0,0x1020bf,0xfe0bf,0xfe0c0,0x1060c0,0x1060bf,0x302041,0x301fc1,0x2fe041,0x2fdfc1,0x70203e,0x701fbe,0x306041,0x305fc1,0x3020c1,0x6fe03e,0x6fdfbe,0x70603e,0x705fbe,0x7020be,0x2fe0c1,0x13060c1,0x3101f40,0x3101f3f,0x30fdf3f,0x6fe0be,0x17060be,0x30fdf40,0x3105f40,0x3105f3f,0x186fa03f,0x180fa040,0x1b0f9fc0,0x1b6f9fbf,0x186fa0bf,0x180fa0c0,0x830a040,0x870a03f,0xb709fbf,0xb309fc0,0x3301f41,0x1b2fdf41,0xb305f41,0xb701f3e,0x1b6fdf3e,0x970213f,0x9302140,0x930a0c0,0x970a0bf,0x196fe13f,0x192fe140,0x1a2fa041,
	14,0x10203f,0x101fc0,0x101fbf,0xfe040,0xfe03f,0xfdfc0,0xfdfbf,0x106040,0x10603f,0x105fc0,0x105fbf,0x1020c0,0x1020bf,0xfe0c0,0x302041,0xfe0bf,0x301fc1,0x1060c0,0x1060bf,0x2fe041,0x2fdfc1,0x306041,0x305fc1,0x3020c1,0x2fe0c1,0x13060c1,0x70203e,0x701fbe,0x6fe03e,0x6fdfbe,0x70603e,0x705fbe,0x3701f3f,0x3101f40,0x30fdf40,0x36fdf3f,0x7020be,0x16fe0be,0x186fa03f,0x180fa040,0x1b0f9fc0,0x3105f40,0x3705f3f,0x1b6f9fbf,0x17060be,0x870a03f,0x810a040,0x3301f41,0x32fdf41,0xb109fc0,0xb709fbf,0x180fa0c0,0x196fa0bf,0x192fa041,0x1b2f9fc1,0x3305f41,0x9302140,0x970213f,0x910a0c0,0x970a0bf,0x930a041,0xb309fc1,0x194fe140,
	15,0x10203f,0x101fc0,0x101fbf,0xfe040,0xfe03f,0xfdfc0,0x106040,0x10603f,0xfdfbf,0x105fc0,0x102041,0x1020c0,0x105fbf,0x1020bf,0x101fc1,0xfe041,0xfe0c0,0xfe0bf,0xfdfc1,0x106041,0x1060c0,0x1060bf,0x105fc1,0x1020c1,0x2fe0c1,0x13060c1,0x70203e,0x3101f40,0x3101f3f,0x3701fbe,0x6fe03e,0x6fdfbe,0x30fdf40,0x36fdf3f,0x3105f40,0x3105f3f,0x70603e,0x3705fbe,0x180fa040,0x186fa03f,0x1b0f9fc0,0x1b6f9fbf,0x7020be,0x16fe0be,0x3101f41,0x32fdf41,0xb305f41,0x810a040,0x870a03f,0x97060be,0xb109fc0,0xb709fbf,0x182fa041,0x182fa0c0,0x196fa0bf,0x1b2f9fc1,0x11302042,0x13301fc2,0xb30a041,0x950a0c0,0x9302140,0x970213f,0x194fe140,
	17,0x101fc0,0x10203f,0xfe040,0xfdfc0,0x101fbf,0x106040,0x102041,0xfe03f,0x101fc1,0x105fc0,0x1020c0,0xfdfbf,0x10603f,0xfe041,0xfdfc1,0x105fbf,0x106041,0x1020bf,0xfe0c0,0x105fc1,0x1060c0,0x1020c1,0x10fe0bf,0x11060bf,0x10fe0c1,0x11060c1,0x3101f40,0x30fdf40,0x3101f3f,0x3105f40,0x3101f41,0x30fdf3f,0x70203e,0x3701fbe,0x180fa040,0x1b0f9fc0,0x30fdf41,0x3105f3f,0x6fe03e,0x186fa03f,0x1b0f9fbf,0x1b6fdfbe,0x70603e,0x3705fbe,0xb305f41,0x910a040,0xb109fc0,0x1302042,0x180fa041,0x1b0f9fc1,0x3301fc2,0x192fe042,0x192fa0c0,0x17020be,0x970a03f,0xb709fbf,0xa10a041,0xa306042,0x1b2fdfc2,0x190fa0bf,0x196fe0be,0x97060be,0x11502140,
	17,0x10203f,0x101fbf,0x101fc0,0xfe040,0xfe03f,0x1020bf,0x1020c0,0x106040,0x10603f,0xfdfbf,0xfdfc0,0x105fc0,0x105fbf,0xfe0bf,0xfe0c0,0x1060c0,0x1060bf,0x302041,0x301fc1,0x2fe041,0x3020c1,0x306041,0x70203e,0x701fbe,0x6fe03e,0x2fdfc1,0x305fc1,0x2fe0c1,0x7020be,0x70603e,0x6fdfbe,0x3060c1,0x705fbe,0x6fe0be,0x7060be,0x3701f3f,0x3101f40,0x180fa040,0x186fa03f,0x186f9fbf,0x180f9fc0,0x1b0fdf40,0x1b6fdf3f,0x3705f3f,0x3105f40,0x830a040,0x870a03f,0x170213f,0x1302140,0x180fa0c0,0x186fa0bf,0x196fe13f,0x192fe140,0x1306140,0x170613f,0xb709fbf,0xb309fc0,0x930a0c0,0x970a0bf,0xb301f41,0x192fa041,0x182f9fc1,0x1b2fdf41,
	17,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0x106040,0x1020c0,0x1020bf,0x10603f,0xfdfc0,0xfdfbf,0x105fc0,0x105fbf,0xfe0c0,0xfe0bf,0x1060c0,0x1060bf,0x302041,0x301fc1,0x2fe041,0x3020c1,0x306041,0x2fdfc1,0x305fc1,0x2fe0c1,0x3060c1,0x70203e,0x6fe03e,0x701fbe,0x70603e,0x7020be,0x6fdfbe,0x705fbe,0x6fe0be,0x7060be,0x3101f40,0x3701f3f,0x180fa040,0x186fa03f,0x180f9fc0,0x1b0fdf40,0x36fdf3f,0x1b6f9fbf,0x180fa0c0,0x186fa0bf,0x1302140,0x170213f,0x830a040,0x3105f40,0x3705f3f,0x870a03f,0xb309fc0,0xb709fbf,0x830a0c0,0x192fe140,0x196fe13f,0x9306140,0x170613f,0x970a0bf,0xb301f41,0x192fa041,0x182f9fc1,0x1b2fdf41,
	17,0x10203f,0x101fc0,0xfe040,0xfe03f,0x101fbf,0x106040,0x1020c0,0xfdfc0,0x1020bf,0x10603f,0x102041,0xfdfbf,0x105fc0,0xfe0c0,0xfe0bf,0x105fbf,0x1060c0,0x101fc1,0xfe041,0x1060bf,0x106041,0x1020c1,0x2fdfc1,0x305fc1,0x2fe0c1,0x3060c1,0x70203e,0x6fe03e,0x701fbe,0x70603e,0x7020be,0x6fdfbe,0x3101f40,0x3701f3f,0x180fa040,0x186fa03f,0x6fe0be,0x705fbe,0x30fdf40,0x1b0f9fc0,0x186f9fbf,0x1b6fdf3f,0x3105f40,0x3705f3f,0x97060be,0x830a040,0x870a03f,0x1302140,0x180fa0c0,0x186fa0bf,0x170213f,0x192fe140,0x192fa041,0x3301f41,0xb309fc0,0xb709fbf,0x850a0c0,0x9506140,0x196fe13f,0x182f9fc1,0x1b2fdf41,0xb305f41,0x12302042,
	16,0x10203f,0x101fc0,0xfe040,0x102041,0x1020c0,0x106040,0x101fbf,0xfe03f,0xfdfc0,0x101fc1,0x105fc0,0x10603f,0x1020bf,0xfe0c0,0xfe041,0xfdfbf,0x1020c1,0x106041,0x1060c0,0x105fbf,0xfe0bf,0xfdfc1,0x105fc1,0x1060bf,0xfe0c1,0x83060c1,0x70203e,0x3101f40,0x180fa040,0x180fa03f,0x186fe03e,0x701fbe,0x3701f3f,0x30fdf40,0x1b0f9fc0,0x180fa041,0x180fa0c0,0x7020be,0x70603e,0x3105f40,0xb101f41,0x9302042,0x1102140,0x930a040,0x6fdfbe,0x36fdf3f,0x1b6f9fbf,0x190fa0bf,0x196fe0be,0x170213f,0x196fe140,0x182f9fc1,0x1b2fdf41,0xb301fc2,0x1a2fe042,0x192fa0c1,0x8705fbe,0xb705f3f,0xb309fc0,0xa70a03f,0x97060be,0x9706140,0x11302141
	};

	/* \brief A class representing a single Voronoi cell.
	 *
	 * This class represents a single Voronoi cell, as a collection of vertices
	 * that are connected by edges. The class contains routines for initializing
	 * the Voronoi cell to be simple shapes such as a box, tetrahedron, or octahedron.
	 * It the contains routines for recomputing the cell based on cutting it
	 * by a plane, which forms the key routine for the Voronoi cell computation.
	 * It contains numerous routine for computing statistics about the Voronoi cell,
	 * and it can output the cell in several formats.
	 *
	 * This class is not intended for direct use, but forms the base of the
	 * voronoicell and voronoicell_neighbor classes, which extend it based on
	 * whether neighboring particle ID information needs to be tracked. */
	class voronoicell_base {
	public:
		/* This holds the current size of the arrays ed and nu, which
		 * hold the vertex information. If more vertices are created
		 * than can fit in this array, then it is dynamically extended
		 * using the add_memory_vertices routine. */
		int current_vertices;
		/* This holds the current maximum allowed order of a vertex,
		 * which sets the size of the mem, mep, and mec arrays. If a
		 * vertex is created with more vertices than this, the arrays
		 * are dynamically extended using the add_memory_vorder routine.
		 */
		int current_vertex_order;
		/* This sets the size of the main delete stack. */
		int current_delete_size;
		/* This sets the size of the auxiliary delete stack. */
		int current_delete2_size;
		/* This sets the total number of vertices in the current cell.
		 */
		int p;
		/* This is the index of particular point in the cell, which is
		 * used to start the tracing routines for plane intersection
		 * and cutting. These routines will work starting from any
		 * point, but it's often most efficient to start from the last
		 * point considered, since in many cases, the cell construction
		 * algorithm may consider many planes with similar vectors
		 * concurrently. */
		int up;
		/* This is a two dimensional array that holds information
		 * about the edge connections of the vertices that make up the
		 * cell. The two dimensional array is not allocated in the
		 * usual method. To account for the fact the different vertices
		 * have different orders, and thus require different amounts of
		 * storage, the elements of ed[i] point to one-dimensional
		 * arrays in the mep[] array of different sizes.
		 *
		 * More specifically, if vertex i has order m, then ed[i]
		 * points to a one-dimensional array in mep[m] that has 2*m+1
		 * entries. The first m elements hold the neighboring edges, so
		 * that the jth edge of vertex i is held in ed[i][j]. The next
		 * m elements hold a table of relations which is redundant but
		 * helps speed up the computation. It satisfies the relation
		 * ed[ed[i][j]][ed[i][m+j]]=i. The final entry holds a back
		 * pointer, so that ed[i+2*m]=i. The back pointers are used
		 * when rearranging the memory. */
		int** ed;
		/* This array holds the order of the vertices in the Voronoi
		 * cell. This array is dynamically allocated, with its current
		 * size held by current_vertices. */
		int* nu;
		/* This in an array with size 3*current_vertices for holding
		 * the positions of the vertices. */
		double* pts;
		voronoicell_base() :
			current_vertices(init_vertices), current_vertex_order(init_vertex_order),
			current_delete_size(init_delete_size), current_delete2_size(init_delete2_size),
			ed(new int* [current_vertices]), nu(new int[current_vertices]),
			pts(new double[3 * current_vertices]), mem(new int[current_vertex_order]),
			mec(new int[current_vertex_order]), mep(new int* [current_vertex_order]),
			ds(new int[current_delete_size]), stacke(ds + current_delete_size),
			ds2(new int[current_delete2_size]), stacke2(ds2 + current_delete_size),
			current_marginal(init_marginal), marg(new int[current_marginal]) {
			int i;
			for (i = 0; i < 3; i++) {
				mem[i] = init_n_vertices; mec[i] = 0;
				mep[i] = new int[init_n_vertices * ((i << 1) + 1)];
			}
			mem[3] = init_3_vertices; mec[3] = 0;
			mep[3] = new int[init_3_vertices * 7];
			for (i = 4; i < current_vertex_order; i++) {
				mem[i] = init_n_vertices; mec[i] = 0;
				mep[i] = new int[init_n_vertices * ((i << 1) + 1)];
			}
		}

		virtual ~voronoicell_base()
		{
			for (int i = current_vertex_order - 1; i >= 0; i--) if (mem[i] > 0) delete[] mep[i];
			delete[] marg;
			delete[] ds2; delete[] ds;
			delete[] mep; delete[] mec;
			delete[] mem; delete[] pts;
			delete[] nu; delete[] ed;
		}

		void init_base(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
		{
			for (int i = 0; i < current_vertex_order; i++) mec[i] = 0; up = 0;
			mec[3] = p = 8; xmin *= 2; xmax *= 2; ymin *= 2; ymax *= 2; zmin *= 2; zmax *= 2;
			*pts = xmin; pts[1] = ymin; pts[2] = zmin;
			pts[3] = xmax; pts[4] = ymin; pts[5] = zmin;
			pts[6] = xmin; pts[7] = ymax; pts[8] = zmin;
			pts[9] = xmax; pts[10] = ymax; pts[11] = zmin;
			pts[12] = xmin; pts[13] = ymin; pts[14] = zmax;
			pts[15] = xmax; pts[16] = ymin; pts[17] = zmax;
			pts[18] = xmin; pts[19] = ymax; pts[20] = zmax;
			pts[21] = xmax; pts[22] = ymax; pts[23] = zmax;
			int* q = mep[3];
			*q = 1; q[1] = 4; q[2] = 2; q[3] = 2; q[4] = 1; q[5] = 0; q[6] = 0;
			q[7] = 3; q[8] = 5; q[9] = 0; q[10] = 2; q[11] = 1; q[12] = 0; q[13] = 1;
			q[14] = 0; q[15] = 6; q[16] = 3; q[17] = 2; q[18] = 1; q[19] = 0; q[20] = 2;
			q[21] = 2; q[22] = 7; q[23] = 1; q[24] = 2; q[25] = 1; q[26] = 0; q[27] = 3;
			q[28] = 6; q[29] = 0; q[30] = 5; q[31] = 2; q[32] = 1; q[33] = 0; q[34] = 4;
			q[35] = 4; q[36] = 1; q[37] = 7; q[38] = 2; q[39] = 1; q[40] = 0; q[41] = 5;
			q[42] = 7; q[43] = 2; q[44] = 4; q[45] = 2; q[46] = 1; q[47] = 0; q[48] = 6;
			q[49] = 5; q[50] = 3; q[51] = 6; q[52] = 2; q[53] = 1; q[54] = 0; q[55] = 7;
			*ed = q; ed[1] = q + 7; ed[2] = q + 14; ed[3] = q + 21;
			ed[4] = q + 28; ed[5] = q + 35; ed[6] = q + 42; ed[7] = q + 49;
			*nu = nu[1] = nu[2] = nu[3] = nu[4] = nu[5] = nu[6] = nu[7] = 3;
		}

		void init_octahedron_base(double l)
		{
			for (int i = 0; i < current_vertex_order; i++) mec[i] = 0; up = 0;
			mec[4] = p = 6; l *= 2;
			*pts = -l; pts[1] = 0; pts[2] = 0;
			pts[3] = l; pts[4] = 0; pts[5] = 0;
			pts[6] = 0; pts[7] = -l; pts[8] = 0;
			pts[9] = 0; pts[10] = l; pts[11] = 0;
			pts[12] = 0; pts[13] = 0; pts[14] = -l;
			pts[15] = 0; pts[16] = 0; pts[17] = l;
			int* q = mep[4];
			*q = 2; q[1] = 5; q[2] = 3; q[3] = 4; q[4] = 0; q[5] = 0; q[6] = 0; q[7] = 0; q[8] = 0;
			q[9] = 2; q[10] = 4; q[11] = 3; q[12] = 5; q[13] = 2; q[14] = 2; q[15] = 2; q[16] = 2; q[17] = 1;
			q[18] = 0; q[19] = 4; q[20] = 1; q[21] = 5; q[22] = 0; q[23] = 3; q[24] = 0; q[25] = 1; q[26] = 2;
			q[27] = 0; q[28] = 5; q[29] = 1; q[30] = 4; q[31] = 2; q[32] = 3; q[33] = 2; q[34] = 1; q[35] = 3;
			q[36] = 0; q[37] = 3; q[38] = 1; q[39] = 2; q[40] = 3; q[41] = 3; q[42] = 1; q[43] = 1; q[44] = 4;
			q[45] = 0; q[46] = 2; q[47] = 1; q[48] = 3; q[49] = 1; q[50] = 3; q[51] = 3; q[52] = 1; q[53] = 5;
			*ed = q; ed[1] = q + 9; ed[2] = q + 18; ed[3] = q + 27; ed[4] = q + 36; ed[5] = q + 45;
			*nu = nu[1] = nu[2] = nu[3] = nu[4] = nu[5] = 4;
		}

		void init_tetrahedron_base(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3)
		{
			for (int i = 0; i < current_vertex_order; i++) mec[i] = 0; up = 0;
			mec[3] = p = 4;
			*pts = x0 * 2; pts[1] = y0 * 2; pts[2] = z0 * 2;
			pts[3] = x1 * 2; pts[4] = y1 * 2; pts[5] = z1 * 2;
			pts[6] = x2 * 2; pts[7] = y2 * 2; pts[8] = z2 * 2;
			pts[9] = x3 * 2; pts[10] = y3 * 2; pts[11] = z3 * 2;
			int* q = mep[3];
			*q = 1; q[1] = 3; q[2] = 2; q[3] = 0; q[4] = 0; q[5] = 0; q[6] = 0;
			q[7] = 0; q[8] = 2; q[9] = 3; q[10] = 0; q[11] = 2; q[12] = 1; q[13] = 1;
			q[14] = 0; q[15] = 3; q[16] = 1; q[17] = 2; q[18] = 2; q[19] = 1; q[20] = 2;
			q[21] = 0; q[22] = 1; q[23] = 2; q[24] = 1; q[25] = 2; q[26] = 1; q[27] = 3;
			*ed = q; ed[1] = q + 7; ed[2] = q + 14; ed[3] = q + 21;
			*nu = nu[1] = nu[2] = nu[3] = 3;
		}

		void translate(double x, double y, double z)
		{
			x *= 2; y *= 2; z *= 2;
			double* ptsp = pts;
			while (ptsp < pts + 3 * p) {
				*(ptsp++) = x; *(ptsp++) = y; *(ptsp++) = z;
			}
		}

		double volume()
		{
			const double fe = 1 / 48.0;
			double vol = 0;
			int i, j, k, l, m, n;
			double ux, uy, uz, vx, vy, vz, wx, wy, wz;
			for (i = 1; i < p; i++) {
				ux = *pts - pts[3 * i];
				uy = pts[1] - pts[3 * i + 1];
				uz = pts[2] - pts[3 * i + 2];
				for (j = 0; j < nu[i]; j++) {
					k = ed[i][j];
					if (k >= 0) {
						ed[i][j] = -1 - k;
						l = cycle_up(ed[i][nu[i] + j], k);
						vx = pts[3 * k] - *pts;
						vy = pts[3 * k + 1] - pts[1];
						vz = pts[3 * k + 2] - pts[2];
						m = ed[k][l]; ed[k][l] = -1 - m;
						while (m != i) {
							n = cycle_up(ed[k][nu[k] + l], m);
							wx = pts[3 * m] - *pts;
							wy = pts[3 * m + 1] - pts[1];
							wz = pts[3 * m + 2] - pts[2];
							vol += ux * vy * wz + uy * vz * wx + uz * vx * wy - uz * vy * wx - uy * vx * wz - ux * vz * wy;
							k = m; l = n; vx = wx; vy = wy; vz = wz;
							m = ed[k][l]; ed[k][l] = -1 - m;
						}
					}
				}
			}
			reset_edges();
			return vol * fe;
		}

		double max_radius_squared()
		{
			double r, s, * ptsp = pts + 3, * ptse = pts + 3 * p;
			r = *pts * (*pts) + pts[1] * pts[1] + pts[2] * pts[2];
			while (ptsp < ptse) {
				s = *ptsp * (*ptsp); ptsp++;
				s += *ptsp * (*ptsp); ptsp++;
				s += *ptsp * (*ptsp); ptsp++;
				if (s > r) r = s;
			}
			return r;
		}

		double total_edge_distance()
		{
			int i, j, k;
			double dis = 0, dx, dy, dz;
			for (i = 0; i < p - 1; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k > i) {
					dx = pts[3 * k] - pts[3 * i];
					dy = pts[3 * k + 1] - pts[3 * i + 1];
					dz = pts[3 * k + 2] - pts[3 * i + 2];
					dis += sqrt(dx * dx + dy * dy + dz * dz);
				}
			}
			return 0.5 * dis;
		}

		double surface_area()
		{
			double area = 0;
			int i, j, k, l, m, n;
			double ux, uy, uz, vx, vy, vz, wx, wy, wz;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					m = ed[k][l]; ed[k][l] = -1 - m;
					while (m != i) {
						n = cycle_up(ed[k][nu[k] + l], m);
						ux = pts[3 * k] - pts[3 * i];
						uy = pts[3 * k + 1] - pts[3 * i + 1];
						uz = pts[3 * k + 2] - pts[3 * i + 2];
						vx = pts[3 * m] - pts[3 * i];
						vy = pts[3 * m + 1] - pts[3 * i + 1];
						vz = pts[3 * m + 2] - pts[3 * i + 2];
						wx = uy * vz - uz * vy;
						wy = uz * vx - ux * vz;
						wz = ux * vy - uy * vx;
						area += sqrt(wx * wx + wy * wy + wz * wz);
						k = m; l = n;
						m = ed[k][l]; ed[k][l] = -1 - m;
					}
				}
			}
			reset_edges();
			return 0.125 * area;
		}

		void centroid(double& cx, double& cy, double& cz)
		{
			double tvol, vol = 0; cx = cy = cz = 0;
			int i, j, k, l, m, n;
			double ux, uy, uz, vx, vy, vz, wx, wy, wz;
			for (i = 1; i < p; i++) {
				ux = *pts - pts[3 * i];
				uy = pts[1] - pts[3 * i + 1];
				uz = pts[2] - pts[3 * i + 2];
				for (j = 0; j < nu[i]; j++) {
					k = ed[i][j];
					if (k >= 0) {
						ed[i][j] = -1 - k;
						l = cycle_up(ed[i][nu[i] + j], k);
						vx = pts[3 * k] - *pts;
						vy = pts[3 * k + 1] - pts[1];
						vz = pts[3 * k + 2] - pts[2];
						m = ed[k][l]; ed[k][l] = -1 - m;
						while (m != i) {
							n = cycle_up(ed[k][nu[k] + l], m);
							wx = pts[3 * m] - *pts;
							wy = pts[3 * m + 1] - pts[1];
							wz = pts[3 * m + 2] - pts[2];
							tvol = ux * vy * wz + uy * vz * wx + uz * vx * wy - uz * vy * wx - uy * vx * wz - ux * vz * wy;
							vol += tvol;
							cx += (wx + vx - ux) * tvol;
							cy += (wy + vy - uy) * tvol;
							cz += (wz + vz - uz) * tvol;
							k = m; l = n; vx = wx; vy = wy; vz = wz;
							m = ed[k][l]; ed[k][l] = -1 - m;
						}
					}
				}
			}
			reset_edges();
			if (vol > tolerance_sq) {
				vol = 0.125 / vol;
				cx = cx * vol + 0.5 * (*pts);
				cy = cy * vol + 0.5 * pts[1];
				cz = cz * vol + 0.5 * pts[2];
			}
			else cx = cy = cz = 0;
		}

		int number_of_faces()
		{
			int i, j, k, l, m, s = 0;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					s++;
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						m = ed[k][l];
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);

				}
			}
			reset_edges();
			return s;
		}

		int number_of_edges();
		void vertex_orders(std::vector<int>& v)
		{
			v.resize(p);
			for (int i = 0; i < p; i++) v[i] = nu[i];
		}

		void vertices(std::vector<double>& v)
		{
			v.resize(3 * p);
			double* ptsp = pts;
			for (int i = 0; i < 3 * p; i += 3) {
				v[i] = *(ptsp++) * 0.5;
				v[i + 1] = *(ptsp++) * 0.5;
				v[i + 2] = *(ptsp++) * 0.5;
			}
		}

		void vertices(double x, double y, double z, std::vector<double>& v)
		{
			v.resize(3 * p);
			double* ptsp = pts;
			for (int i = 0; i < 3 * p; i += 3) {
				v[i] = x + *(ptsp++) * 0.5;
				v[i + 1] = y + *(ptsp++) * 0.5;
				v[i + 2] = z + *(ptsp++) * 0.5;
			}
		}

		void face_areas(std::vector<double>& v)
		{
			double area;
			v.clear();
			int i, j, k, l, m, n;
			double ux, uy, uz, vx, vy, vz, wx, wy, wz;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					area = 0;
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					m = ed[k][l]; ed[k][l] = -1 - m;
					while (m != i) {
						n = cycle_up(ed[k][nu[k] + l], m);
						ux = pts[3 * k] - pts[3 * i];
						uy = pts[3 * k + 1] - pts[3 * i + 1];
						uz = pts[3 * k + 2] - pts[3 * i + 2];
						vx = pts[3 * m] - pts[3 * i];
						vy = pts[3 * m + 1] - pts[3 * i + 1];
						vz = pts[3 * m + 2] - pts[3 * i + 2];
						wx = uy * vz - uz * vy;
						wy = uz * vx - ux * vz;
						wz = ux * vy - uy * vx;
						area += sqrt(wx * wx + wy * wy + wz * wz);
						k = m; l = n;
						m = ed[k][l]; ed[k][l] = -1 - m;
					}
					v.push_back(0.125 * area);
				}
			}
			reset_edges();
		}
		void face_orders(std::vector<int>& v)
		{
			int i, j, k, l, m, q;
			v.clear();
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					q = 1;
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						q++;
						m = ed[k][l];
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
					v.push_back(q);;
				}
			}
			reset_edges();
		}

		void face_freq_table(std::vector<int>& v)
		{
			int i, j, k, l, m, q;
			v.clear();
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					q = 1;
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						q++;
						m = ed[k][l];
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
					if ((unsigned int)q >= v.size()) v.resize(q + 1, 0);
					v[q]++;
				}
			}
			reset_edges();
		}

		void face_vertices(std::vector<int>& v)
		{
			int i, j, k, l, m, vp(0), vn;
			v.clear();
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					v.push_back(0);
					v.push_back(i);
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						v.push_back(k);
						m = ed[k][l];
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
					vn = (int)v.size();
					v[vp] = vn - vp - 1;
					vp = vn;
				}
			}
			reset_edges();
		}

		void face_perimeters(std::vector<double>& v)
		{
			v.clear();
			int i, j, k, l, m;
			double dx, dy, dz, perim;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					dx = pts[3 * k] - pts[3 * i];
					dy = pts[3 * k + 1] - pts[3 * i + 1];
					dz = pts[3 * k + 2] - pts[3 * i + 2];
					perim = sqrt(dx * dx + dy * dy + dz * dz);
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						m = ed[k][l];
						dx = pts[3 * m] - pts[3 * k];
						dy = pts[3 * m + 1] - pts[3 * k + 1];
						dz = pts[3 * m + 2] - pts[3 * k + 2];
						perim += sqrt(dx * dx + dy * dy + dz * dz);
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
					v.push_back(0.5 * perim);
				}
			}
			reset_edges();
		}

		void normals(std::vector<double>& v)
		{
			int i, j, k;
			v.clear();
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) normals_search(v, i, j, k);
			}
			reset_edges();
		}

		template<class vc_class>
		bool nplane(vc_class& vc, double x, double y, double z, double rsq, int p_id)
		{
			int count = 0, i, j, k, lp = up, cp, qp, rp, *stackp(ds), *stackp2(ds2), * dsp;
			int us = 0, ls = 0, qs, iqs, cs, uw, qw, lw;
			int* edp, * edd;
			double u, l = 0, r, q; bool complicated_setup = false, new_double_edge = false, double_edge = false;

			// Initialize the safe testing routine
			n_marg = 0; px = x; py = y; pz = z; prsq = rsq;

			// Test approximately sqrt(n)/4 points for their proximity to the plane
			// and keep the one which is closest
			uw = m_test(up, u);

			// Starting from an initial guess, we now move from vertex to vertex,
			// to try and find an edge which intersects the cutting plane,
			// or a vertex which is on the plane
			bool failed = false;
			if (uw == 1) {

				// The test point is inside the cutting plane.
				us = 0;
				do {
					lp = ed[up][us];
					lw = m_test(lp, l);
					if (l < u) break;
					us++;
				} while (us < nu[up]);

				if (us == nu[up]) {
					return false;
				}

				ls = ed[up][nu[up] + us];
				while (lw == 1) {
					if (++count >= p) {
						failed = true;
						break;
					}
					u = l; up = lp;
					for (us = 0; us < ls; us++) {
						lp = ed[up][us];
						lw = m_test(lp, l);
						if (l < u) break;
					}
					if (us == ls) {
						us++;
						while (us < nu[up]) {
							lp = ed[up][us];
							lw = m_test(lp, l);
							if (l < u) break;
							us++;
						}
						if (us == nu[up]) {
							return false;
						}
					}
					ls = ed[up][nu[up] + us];
				}

				// If the last point in the iteration is within the
				// plane, we need to do the complicated setup
				// routine. Otherwise, we use the regular iteration.
				if (!failed) {
					if (lw == 0) {
						up = lp;
						complicated_setup = true;
					}
					else complicated_setup = false;
				}
			}
			else if (uw == -1) {
				us = 0;
				do {
					qp = ed[up][us];
					qw = m_test(qp, q);
					if (u < q) break;
					us++;
				} while (us < nu[up]);
				if (us == nu[up]) return true;

				while (qw == -1) {
					qs = ed[up][nu[up] + us];
					if (++count >= p) {
						failed = true;
						break;
					}
					u = q; up = qp;
					for (us = 0; us < qs; us++) {
						qp = ed[up][us];
						qw = m_test(qp, q);
						if (u < q) break;
					}
					if (us == qs) {
						us++;
						while (us < nu[up]) {
							qp = ed[up][us];
							qw = m_test(qp, q);
							if (u < q) break;
							us++;
						}
						if (us == nu[up]) return true;
					}
				}
				if (!failed) {
					if (qw == 1) {
						lp = up; ls = us; l = u;
						up = qp; us = ed[lp][nu[lp] + ls]; u = q;
						complicated_setup = false;
					}
					else {
						up = qp;
						complicated_setup = true;
					}
				}
			}
			else {

				// Our original test point was on the plane, so we
				// automatically head for the complicated setup
				// routine
				complicated_setup = true;
			}

			if (failed) {
				// This routine is a fall-back, in case floating point errors
				// cause the usual search routine to fail. In the fall-back
				// routine, we just test every edge to find one straddling
				// the plane.
				qw = 1; lw = 0;
				for (qp = 0; qp < p; qp++) {
					qw = m_test(qp, q);
					if (qw == 1) {

						// The point is inside the cutting space. Now
						// see if we can find a neighbor which isn't.
						for (us = 0; us < nu[qp]; us++) {
							lp = ed[qp][us];
							if (lp < qp) {
								lw = m_test(lp, l);
								if (lw != 1) break;
							}
						}
						if (us < nu[qp]) {
							up = qp;
							if (lw == 0) {
								complicated_setup = true;
							}
							else {
								complicated_setup = false;
								u = q;
								ls = ed[up][nu[up] + us];
							}
							break;
						}
					}
					else if (qw == -1) {

						// The point is outside the cutting space. See
						// if we can find a neighbor which isn't.
						for (ls = 0; ls < nu[qp]; ls++) {
							up = ed[qp][ls];
							if (up < qp) {
								uw = m_test(up, u);
								if (uw != -1) break;
							}
						}
						if (ls < nu[qp]) {
							if (uw == 0) {
								up = qp;
								complicated_setup = true;
							}
							else {
								complicated_setup = false;
								lp = qp; l = q;
								us = ed[lp][nu[lp] + ls];
							}
							break;
						}
					}
					else {

						// The point is in the plane, so we just
						// proceed with the complicated setup routine
						up = qp;
						complicated_setup = true;
						break;
					}
				}
				if (qp == p) return qw == -1 ? true : false;
			}

			// We're about to add the first point of the new facet. In either
			// routine, we have to add a point, so first check there's space for
			// it.
			if (p == current_vertices) add_memory_vertices(vc);

			if (complicated_setup) {

				// We want to be strict about reaching the conclusion that the
				// cell is entirely within the cutting plane. It's not enough
				// to find a vertex that has edges which are all inside or on
				// the plane. If the vertex has neighbors that are also on the
				// plane, we should check those too.
				if (!search_for_outside_edge(vc, up)) return false;

				// The search algorithm found a point which is on the cutting
				// plane. We leave that point in place, and create a new one at
				// the same location.
				pts[3 * p] = pts[3 * up];
				pts[3 * p + 1] = pts[3 * up + 1];
				pts[3 * p + 2] = pts[3 * up + 2];

				// Search for a collection of edges of the test vertex which
				// are outside of the cutting space. Begin by testing the
				// zeroth edge.
				i = 0;
				lp = *ed[up];
				lw = m_test(lp, l);
				if (lw != -1) {

					// The first edge is either inside the cutting space,
					// or lies within the cutting plane. Test the edges
					// sequentially until we find one that is outside.
					rp = lw;
					do {
						i++;

						// If we reached the last edge with no luck
						// then all of the vertices are inside
						// or on the plane, so the cell is completely
						// deleted
						if (i == nu[up]) return false;
						lp = ed[up][i];
						lw = m_test(lp, l);
					} while (lw != -1);
					j = i + 1;

					// We found an edge outside the cutting space. Keep
					// moving through these edges until we find one that's
					// inside or on the plane.
					while (j < nu[up]) {
						lp = ed[up][j];
						lw = m_test(lp, l);
						if (lw != -1) break;
						j++;
					}

					// Compute the number of edges for the new vertex. In
					// general it will be the number of outside edges
					// found, plus two. But we need to recognize the
					// special case when all but one edge is outside, and
					// the remaining one is on the plane. For that case we
					// have to reduce the edge count by one to prevent
					// doubling up.
					if (j == nu[up] && i == 1 && rp == 0) {
						nu[p] = nu[up];
						double_edge = true;
					}
					else nu[p] = j - i + 2;
					k = 1;

					// Add memory for the new vertex if needed, and
					// initialize
					while (nu[p] >= current_vertex_order) add_memory_vorder(vc);
					if (mec[nu[p]] == mem[nu[p]]) add_memory(vc, nu[p], stackp2);
					vc.n_set_pointer(p, nu[p]);
					ed[p] = mep[nu[p]] + ((nu[p] << 1) + 1) * mec[nu[p]]++;
					ed[p][nu[p] << 1] = p;

					// Copy the edges of the original vertex into the new
					// one. Delete the edges of the original vertex, and
					// update the relational table.
					us = cycle_down(i, up);
					while (i < j) {
						qp = ed[up][i];
						qs = ed[up][nu[up] + i];
						vc.n_copy(p, k, up, i);
						ed[p][k] = qp;
						ed[p][nu[p] + k] = qs;
						ed[qp][qs] = p;
						ed[qp][nu[qp] + qs] = k;
						ed[up][i] = -1;
						i++; k++;
					}
					qs = i == nu[up] ? 0 : i;
				}
				else {

					// In this case, the zeroth edge is outside the cutting
					// plane. Begin by searching backwards from the last
					// edge until we find an edge which isn't outside.
					i = nu[up] - 1;
					lp = ed[up][i];
					lw = m_test(lp, l);
					while (lw == -1) {
						i--;

						// If i reaches zero, then we have a point in
						// the plane all of whose edges are outside
						// the cutting space, so we just exit
						if (i == 0) return true;
						lp = ed[up][i];
						lw = m_test(lp, l);
					}

					// Now search forwards from zero
					j = 1;
					qp = ed[up][j];
					qw = m_test(qp, q);
					while (qw == -1) {
						j++;
						qp = ed[up][j];
						qw = m_test(qp, l);
					}

					// Compute the number of edges for the new vertex. In
					// general it will be the number of outside edges
					// found, plus two. But we need to recognize the
					// special case when all but one edge is outside, and
					// the remaining one is on the plane. For that case we
					// have to reduce the edge count by one to prevent
					// doubling up.
					if (i == j && qw == 0) {
						double_edge = true;
						nu[p] = nu[up];
					}
					else {
						nu[p] = nu[up] - i + j + 1;
					}

					// Add memory to store the vertex if it doesn't exist
					// already
					k = 1;
					while (nu[p] >= current_vertex_order) add_memory_vorder(vc);
					if (mec[nu[p]] == mem[nu[p]]) add_memory(vc, nu[p], stackp2);

					// Copy the edges of the original vertex into the new
					// one. Delete the edges of the original vertex, and
					// update the relational table.
					vc.n_set_pointer(p, nu[p]);
					ed[p] = mep[nu[p]] + ((nu[p] << 1) + 1) * mec[nu[p]]++;
					ed[p][nu[p] << 1] = p;
					us = i++;
					while (i < nu[up]) {
						qp = ed[up][i];
						qs = ed[up][nu[up] + i];
						vc.n_copy(p, k, up, i);
						ed[p][k] = qp;
						ed[p][nu[p] + k] = qs;
						ed[qp][qs] = p;
						ed[qp][nu[qp] + qs] = k;
						ed[up][i] = -1;
						i++; k++;
					}
					i = 0;
					while (i < j) {
						qp = ed[up][i];
						qs = ed[up][nu[up] + i];
						vc.n_copy(p, k, up, i);
						ed[p][k] = qp;
						ed[p][nu[p] + k] = qs;
						ed[qp][qs] = p;
						ed[qp][nu[qp] + qs] = k;
						ed[up][i] = -1;
						i++; k++;
					}
					qs = j;
				}
				if (!double_edge) {
					vc.n_copy(p, k, up, qs);
					vc.n_set(p, 0, p_id);
				}
				else vc.n_copy(p, 0, up, qs);

				// Add this point to the auxiliary delete stack
				if (stackp2 == stacke2) add_memory_ds2(stackp2);
				*(stackp2++) = up;

				// Look at the edges on either side of the group that was
				// detected. We're going to commence facet computation by
				// moving along one of them. We are going to end up coming back
				// along the other one.
				cs = k;
				qp = up; q = u;
				i = ed[up][us];
				us = ed[up][nu[up] + us];
				up = i;
				ed[qp][nu[qp] << 1] = -p;

			}
			else {

				// The search algorithm found an intersected edge between the
				// points lp and up. Create a new vertex between them which
				// lies on the cutting plane. Since u and l differ by at least
				// the tolerance, this division should never screw up.
				if (stackp == stacke) add_memory_ds(stackp);
				*(stackp++) = up;
				r = u / (u - l); l = 1 - r;
				pts[3 * p] = pts[3 * lp] * r + pts[3 * up] * l;
				pts[3 * p + 1] = pts[3 * lp + 1] * r + pts[3 * up + 1] * l;
				pts[3 * p + 2] = pts[3 * lp + 2] * r + pts[3 * up + 2] * l;

				// This point will always have three edges. Connect one of them
				// to lp.
				nu[p] = 3;
				if (mec[3] == mem[3]) add_memory(vc, 3, stackp2);
				vc.n_set_pointer(p, 3);
				vc.n_set(p, 0, p_id);
				vc.n_copy(p, 1, up, us);
				vc.n_copy(p, 2, lp, ls);
				ed[p] = mep[3] + 7 * mec[3]++;
				ed[p][6] = p;
				ed[up][us] = -1;
				ed[lp][ls] = p;
				ed[lp][nu[lp] + ls] = 1;
				ed[p][1] = lp;
				ed[p][nu[p] + 1] = ls;
				cs = 2;

				// Set the direction to move in
				qs = cycle_up(us, up);
				qp = up; q = u;
			}

			// When the code reaches here, we have initialized the first point, and
			// we have a direction for moving it to construct the rest of the facet
			cp = p; rp = p; p++;
			while (qp != up || qs != us) {

				// We're currently tracing round an intersected facet. Keep
				// moving around it until we find a point or edge which
				// intersects the plane.
				lp = ed[qp][qs];
				lw = m_test(lp, l);

				if (lw == 1) {

					// The point is still in the cutting space. Just add it
					// to the delete stack and keep moving.
					qs = cycle_up(ed[qp][nu[qp] + qs], lp);
					qp = lp;
					q = l;
					if (stackp == stacke) add_memory_ds(stackp);
					*(stackp++) = qp;

				}
				else if (lw == -1) {

					// The point is outside of the cutting space, so we've
					// found an intersected edge. Introduce a regular point
					// at the point of intersection. Connect it to the
					// point we just tested. Also connect it to the previous
					// new point in the facet we're constructing.
					if (p == current_vertices) add_memory_vertices(vc);
					r = q / (q - l); l = 1 - r;
					pts[3 * p] = pts[3 * lp] * r + pts[3 * qp] * l;
					pts[3 * p + 1] = pts[3 * lp + 1] * r + pts[3 * qp + 1] * l;
					pts[3 * p + 2] = pts[3 * lp + 2] * r + pts[3 * qp + 2] * l;
					nu[p] = 3;
					if (mec[3] == mem[3]) add_memory(vc, 3, stackp2);
					ls = ed[qp][qs + nu[qp]];
					vc.n_set_pointer(p, 3);
					vc.n_set(p, 0, p_id);
					vc.n_copy(p, 1, qp, qs);
					vc.n_copy(p, 2, lp, ls);
					ed[p] = mep[3] + 7 * mec[3]++;
					*ed[p] = cp;
					ed[p][1] = lp;
					ed[p][3] = cs;
					ed[p][4] = ls;
					ed[p][6] = p;
					ed[lp][ls] = p;
					ed[lp][nu[lp] + ls] = 1;
					ed[cp][cs] = p;
					ed[cp][nu[cp] + cs] = 0;
					ed[qp][qs] = -1;
					qs = cycle_up(qs, qp);
					cp = p++;
					cs = 2;
				}
				else {

					// We've found a point which is on the cutting plane.
					// We're going to introduce a new point right here, but
					// first we need to figure out the number of edges it
					// has.
					if (p == current_vertices) add_memory_vertices(vc);

					// If the previous vertex detected a double edge, our
					// new vertex will have one less edge.
					k = double_edge ? 0 : 1;
					qs = ed[qp][nu[qp] + qs];
					qp = lp;
					iqs = qs;

					// Start testing the edges of the current point until
					// we find one which isn't outside the cutting space
					do {
						k++;
						qs = cycle_up(qs, qp);
						lp = ed[qp][qs];
						lw = m_test(lp, l);
					} while (lw == -1);

					// Now we need to find out whether this marginal vertex
					// we are on has been visited before, because if that's
					// the case, we need to add vertices to the existing
					// new vertex, rather than creating a fresh one. We also
					// need to figure out whether we're in a case where we
					// might be creating a duplicate edge.
					j = -ed[qp][nu[qp] << 1];
					if (qp == up && qs == us) {

						// If we're heading into the final part of the
						// new facet, then we never worry about the
						// duplicate edge calculation.
						new_double_edge = false;
						if (j > 0) k += nu[j];
					}
					else {
						if (j > 0) {

							// This vertex was visited before, so
							// count those vertices to the ones we
							// already have.
							k += nu[j];

							// The only time when we might make a
							// duplicate edge is if the point we're
							// going to move to next is also a
							// marginal point, so test for that
							// first.
							if (lw == 0) {

								// Now see whether this marginal point
								// has been visited before.
								i = -ed[lp][nu[lp] << 1];
								if (i > 0) {

									// Now see if the last edge of that other
									// marginal point actually ends up here.
									if (ed[i][nu[i] - 1] == j) {
										new_double_edge = true;
										k -= 1;
									}
									else new_double_edge = false;
								}
								else {

									// That marginal point hasn't been visited
									// before, so we probably don't have to worry
									// about duplicate edges, except in the
									// case when that's the way into the end
									// of the facet, because that way always creates
									// an edge.
									if (j == rp && lp == up && ed[qp][nu[qp] + qs] == us) {
										new_double_edge = true;
										k -= 1;
									}
									else new_double_edge = false;
								}
							}
							else new_double_edge = false;
						}
						else {

							// The vertex hasn't been visited
							// before, but let's see if it's
							// marginal
							if (lw == 0) {

								// If it is, we need to check
								// for the case that it's a
								// small branch, and that we're
								// heading right back to where
								// we came from
								i = -ed[lp][nu[lp] << 1];
								if (i == cp) {
									new_double_edge = true;
									k -= 1;
								}
								else new_double_edge = false;
							}
							else new_double_edge = false;
						}
					}

					// k now holds the number of edges of the new vertex
					// we are forming. Add memory for it if it doesn't exist
					// already.
					while (k >= current_vertex_order) add_memory_vorder(vc);
					if (mec[k] == mem[k]) add_memory(vc, k, stackp2);

					// Now create a new vertex with order k, or augment
					// the existing one
					if (j > 0) {

						// If we're augmenting a vertex but we don't
						// actually need any more edges, just skip this
						// routine to avoid memory confusion
						if (nu[j] != k) {
							// Allocate memory and copy the edges
							// of the previous instance into it
							vc.n_set_aux1(k);
							edp = mep[k] + ((k << 1) + 1) * mec[k]++;
							i = 0;
							while (i < nu[j]) {
								vc.n_copy_aux1(j, i);
								edp[i] = ed[j][i];
								edp[k + i] = ed[j][nu[j] + i];
								i++;
							}
							edp[k << 1] = j;

							// Remove the previous instance with
							// fewer vertices from the memory
							// structure
							edd = mep[nu[j]] + ((nu[j] << 1) + 1) * --mec[nu[j]];
							if (edd != ed[j]) {
								for (lw = 0; lw <= (nu[j] << 1); lw++) ed[j][lw] = edd[lw];
								vc.n_set_aux2_copy(j, nu[j]);
								vc.n_copy_pointer(edd[nu[j] << 1], j);
								ed[edd[nu[j] << 1]] = ed[j];
							}
							vc.n_set_to_aux1(j);
							ed[j] = edp;
						}
						else i = nu[j];
					}
					else {

						// Allocate a new vertex of order k
						vc.n_set_pointer(p, k);
						ed[p] = mep[k] + ((k << 1) + 1) * mec[k]++;
						ed[p][k << 1] = p;
						if (stackp2 == stacke2) add_memory_ds2(stackp2);
						*(stackp2++) = qp;
						pts[3 * p] = pts[3 * qp];
						pts[3 * p + 1] = pts[3 * qp + 1];
						pts[3 * p + 2] = pts[3 * qp + 2];
						ed[qp][nu[qp] << 1] = -p;
						j = p++;
						i = 0;
					}
					nu[j] = k;

					// Unless the previous case was a double edge, connect
					// the first available edge of the new vertex to the
					// last one in the facet
					if (!double_edge) {
						ed[j][i] = cp;
						ed[j][nu[j] + i] = cs;
						vc.n_set(j, i, p_id);
						ed[cp][cs] = j;
						ed[cp][nu[cp] + cs] = i;
						i++;
					}

					// Copy in the edges of the underlying vertex,
					// and do one less if this was a double edge
					qs = iqs;
					while (i < (new_double_edge ? k : k - 1)) {
						qs = cycle_up(qs, qp);
						lp = ed[qp][qs]; ls = ed[qp][nu[qp] + qs];
						vc.n_copy(j, i, qp, qs);
						ed[j][i] = lp;
						ed[j][nu[j] + i] = ls;
						ed[lp][ls] = j;
						ed[lp][nu[lp] + ls] = i;
						ed[qp][qs] = -1;
						i++;
					}
					qs = cycle_up(qs, qp);
					cs = i;
					cp = j;
					vc.n_copy(j, new_double_edge ? 0 : cs, qp, qs);

					// Update the double_edge flag, to pass it
					// to the next instance of this routine
					double_edge = new_double_edge;
				}
			}

			// Connect the final created vertex to the initial one
			ed[cp][cs] = rp;
			*ed[rp] = cp;
			ed[cp][nu[cp] + cs] = 0;
			ed[rp][nu[rp]] = cs;

			// Delete points: first, remove any duplicates
			dsp = ds;
			while (dsp < stackp) {
				j = *dsp;
				if (ed[j][nu[j]] != -1) {
					ed[j][nu[j]] = -1;
					dsp++;
				}
				else *dsp = *(--stackp);
			}

			// Add the points in the auxiliary delete stack,
			// and reset their back pointers
			for (dsp = ds2; dsp < stackp2; dsp++) {
				j = *dsp;
				ed[j][nu[j] << 1] = j;
				if (ed[j][nu[j]] != -1) {
					ed[j][nu[j]] = -1;
					if (stackp == stacke) add_memory_ds(stackp);
					*(stackp++) = j;
				}
			}

			// Scan connections and add in extras
			for (dsp = ds; dsp < stackp; dsp++) {
				cp = *dsp;
				for (edp = ed[cp]; edp < ed[cp] + nu[cp]; edp++) {
					qp = *edp;
					if (qp != -1 && ed[qp][nu[qp]] != -1) {
						if (stackp == stacke) {
							int dis = (int)(stackp - dsp);
							add_memory_ds(stackp);
							dsp = ds + dis;
						}
						*(stackp++) = qp;
						ed[qp][nu[qp]] = -1;
					}
				}
			}
			up = 0;

			// Delete them from the array structure
			while (stackp > ds) {
				--p;
				while (ed[p][nu[p]] == -1) {
					j = nu[p];
					edp = ed[p]; edd = (mep[j] + ((j << 1) + 1) * --mec[j]);
					while (edp < ed[p] + (size_t(j) << 1) + 1) *(edp++) = *(edd++);
					vc.n_set_aux2_copy(p, j);
					vc.n_copy_pointer(ed[p][(j << 1)], p);
					ed[ed[p][(j << 1)]] = ed[p];
					--p;
				}
				up = *(--stackp);
				if (up < p) {

					// Vertex management
					pts[3 * up] = pts[3 * p];
					pts[3 * up + 1] = pts[3 * p + 1];
					pts[3 * up + 2] = pts[3 * p + 2];

					// Memory management
					j = nu[up];
					edp = ed[up]; edd = (mep[j] + ((j << 1) + 1) * --mec[j]);
					while (edp < ed[up] + (size_t(j) << 1) + 1) *(edp++) = *(edd++);
					vc.n_set_aux2_copy(up, j);
					vc.n_copy_pointer(ed[up][j << 1], up);
					vc.n_copy_pointer(up, p);
					ed[ed[up][j << 1]] = ed[up];

					// Edge management
					ed[up] = ed[p];
					nu[up] = nu[p];
					for (i = 0; i < nu[up]; i++) ed[ed[up][i]][ed[up][nu[up] + i]] = up;
					ed[up][nu[up] << 1] = up;
				}
				else up = p++;
			}

			// Check for any vertices of zero order
			if (*mec > 0) 
			{
				assert(false);
				// voro_fatal_error("Zero order vertex formed", VOROPP_INTERNAL_ERROR);
			}

			// Collapse any order 2 vertices and exit
			return collapse_order2(vc);
		}

		bool plane_intersects(double x, double y, double z, double rsq)
		{
			double g = x * pts[3 * up] + y * pts[3 * up + 1] + z * pts[3 * up + 2];
			if (g < rsq) return plane_intersects_track(x, y, z, rsq, g);
			return true;
		}

		bool plane_intersects_guess(double x, double y, double z, double rsq)
		{
			up = 0;
			double g = x * pts[3 * up] + y * pts[3 * up + 1] + z * pts[3 * up + 2];
			if (g < rsq) {
				int ca = 1, cc = p >> 3, mp = 1;
				double m;
				while (ca < cc) {
					m = x * pts[3 * mp] + y * pts[3 * mp + 1] + z * pts[3 * mp + 2];
					if (m > g) {
						if (m > rsq) return true;
						g = m; up = mp;
					}
					ca += mp++;
				}
				return plane_intersects_track(x, y, z, rsq, g);
			}
			return true;
		}

		void construct_relations()
		{
			int i, j, k, l;
			for (i = 0; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				l = 0;
				while (ed[k][l] != i) {
					l++;
					if (l == nu[k])
					{
						assert(false);
					}
				}
				ed[i][nu[i] + j] = l;
			}
		}

		void check_relations()
		{
			int i, j;
			for (i = 0; i < p; i++) for (j = 0; j < nu[i]; j++) if (ed[ed[i][j]][ed[i][nu[i] + j]] != i)
				printf("Relational error at point %d, edge %d.\n", i, j);
		}

		void check_duplicates()
		{
			int i, j, k;
			for (i = 0; i < p; i++) for (j = 1; j < nu[i]; j++) for (k = 0; k < j; k++) if (ed[i][j] == ed[i][k])
				printf("Duplicate edges: (%d,%d) and (%d,%d) [%d]\n", i, j, i, k, ed[i][j]);
		}

		void print_edges();
		/* Returns a list of IDs of neighboring particles
		 * corresponding to each face.
		 * \param[out] v a reference to a vector in which to return the
		 *               results. If no neighbor information is
		 *               available, a blank vector is returned. */
		virtual void neighbors(std::vector<int>& v) { v.clear(); }
		/* This a virtual function that is overridden by a routine to
		 * print the neighboring particle IDs for a given vertex. By
		 * default, when no neighbor information is available, the
		 * routine does nothing.
		 * \param[in] i the vertex to consider. */
		virtual void print_edges_neighbors(int i) {};
		/* This is a simple inline function for picking out the index
		 * of the next edge counterclockwise at the current vertex.
		 * \param[in] a the index of an edge of the current vertex.
		 * \param[in] p the number of the vertex.
		 * \return 0 if a=nu[p]-1, or a+1 otherwise. */
		inline int cycle_up(int a, int vertexIndex) { return a == nu[vertexIndex] - 1 ? 0 : a + 1; }
		/* This is a simple inline function for picking out the index
		 * of the next edge clockwise from the current vertex.
		 * \param[in] a the index of an edge of the current vertex.
		 * \param[in] p the number of the vertex.
		 * \return nu[p]-1 if a=0, or a-1 otherwise. */
		inline int cycle_down(int a, int vertexIndex) { return a == 0 ? nu[vertexIndex] - 1 : a - 1; }
	protected:
		/* This a one dimensional array that holds the current sizes
		 * of the memory allocations for them mep array.*/
		int* mem;
		/* This is a one dimensional array that holds the current
		 * number of vertices of order p that are stored in the mep[p]
		 * array. */
		int* mec;
		/* This is a two dimensional array for holding the information
		 * about the edges of the Voronoi cell. mep[p] is a
		 * one-dimensional array for holding the edge information about
		 * all vertices of order p, with each vertex holding 2*p+1
		 * integers of information. The total number of vertices held
		 * on mep[p] is stored in mem[p]. If the space runs out, the
		 * code allocates more using the add_memory() routine. */
		int** mep;
		inline void reset_edges()
		{
			int i, j;
			for (i = 0; i < p; i++) for (j = 0; j < nu[i]; j++) {
				if (ed[i][j] >= 0)
				{
					assert(false);
					// voro_fatal_error("Edge reset routine found a previously untested edge", VOROPP_INTERNAL_ERROR);
				}
				ed[i][j] = -1 - ed[i][j];
			}
		}

		template<class vc_class>
		void check_memory_for_copy(vc_class& vc, voronoicell_base* vb)
		{
			while (current_vertex_order < vb->current_vertex_order) add_memory_vorder(vc);
			for (int i = 0; i < current_vertex_order; i++) while (mem[i] < vb->mec[i]) add_memory(vc, i, ds2);
			while (current_vertices < vb->p) add_memory_vertices(vc);
		}

		void copy(voronoicell_base* vb)
		{
			int i, j;
			p = vb->p; up = 0;
			for (i = 0; i < current_vertex_order; i++) {
				mec[i] = vb->mec[i];
				for (j = 0; j < mec[i] * (2 * i + 1); j++) mep[i][j] = vb->mep[i][j];
				for (j = 0; j < mec[i] * (2 * i + 1); j += 2 * i + 1) ed[mep[i][j + 2 * i]] = mep[i] + j;
			}
			for (i = 0; i < p; i++) nu[i] = vb->nu[i];
			for (i = 0; i < 3 * p; i++) pts[i] = vb->pts[i];
		}
	private:
		/* This is the delete stack, used to store the vertices which
		 * are going to be deleted during the plane cutting procedure.
		 */
		int* ds, * stacke;
		/* This is the auxiliary delete stack, which has size set by
		 * current_delete2_size. */
		int* ds2, * stacke2;
		/* This stores the current memory allocation for the marginal
		 * cases. */
		int current_marginal;
		/* This stores the total number of marginal points which are
		 * currently in the buffer. */
		int n_marg;
		/* This array contains a list of the marginal points, and also
		 * the outcomes of the marginal tests. */
		int* marg;
		/* The x coordinate of the normal vector to the test plane. */
		double px;
		/* The y coordinate of the normal vector to the test plane. */
		double py;
		/* The z coordinate of the normal vector to the test plane. */
		double pz;
		/* The magnitude of the normal vector to the test plane. */
		double prsq;
		template<class vc_class>
		void add_memory(vc_class& vc, int i, int* stackp2)
		{
			int s = (i << 1) + 1;
			if (mem[i] == 0) {
				vc.n_allocate(i, init_n_vertices);
				mep[i] = new int[init_n_vertices * s];
				mem[i] = init_n_vertices;
#if VOROPP_VERBOSE >=2
				fprintf(stderr, "Order %d vertex memory created\n", i);
#endif
			}
			else {
				int j = 0, k, * l;
				mem[i] <<= 1;
				if (mem[i] > max_n_vertices)
				{
					assert(false);
					// voro_fatal_error("Point memory allocation exceeded absolute maximum", VOROPP_MEMORY_ERROR);
				}
#if VOROPP_VERBOSE >=2
				fprintf(stderr, "Order %d vertex memory scaled up to %d\n", i, mem[i]);
#endif
				assert(mem[i] > 1 && s > 1);
				l = new int[s * mem[i]];
				int m = 0;
				vc.n_allocate_aux1(i);
				while (j < s * mec[i]) {
					k = mep[i][j + (i << 1)];
					if (k >= 0) {
						ed[k] = l + j;
						vc.n_set_to_aux1_offset(k, m);
					}
					else {
						int* dsp;
						for (dsp = ds2; dsp < stackp2; dsp++) {
							if (ed[*dsp] == mep[i] + j) {
								ed[*dsp] = l + j;
								vc.n_set_to_aux1_offset(*dsp, m);
								break;
							}
						}
						if (dsp == stackp2)
						{
							assert(false);
							// voro_fatal_error("Couldn't relocate dangling pointer", VOROPP_INTERNAL_ERROR);
						}
#if VOROPP_VERBOSE >=3
						fputs("Relocated dangling pointer", stderr);
#endif
					}
					for (k = 0; k < s; k++, j++) l[j] = mep[i][j];
					for (k = 0; k < i; k++, m++) vc.n_copy_to_aux1(i, m);
				}
				delete[] mep[i];
				mep[i] = l;
				vc.n_switch_to_aux1(i);
			}
		}

		template<class vc_class>
		void add_memory_vertices(vc_class& vc)
		{
			int i = (current_vertices << 1), j, ** pp, * pnu;
			if (i > max_vertices)
			{
				assert(false);
				// voro_fatal_error("Vertex memory allocation exceeded absolute maximum", VOROPP_MEMORY_ERROR);
			}
#if VOROPP_VERBOSE >=2
			fprintf(stderr, "Vertex memory scaled up to %d\n", i);
#endif
			double* ppts;
			pp = new int* [i];
			for (j = 0; j < current_vertices; j++) pp[j] = ed[j];
			delete[] ed; ed = pp;
			vc.n_add_memory_vertices(i);
			pnu = new int[i];
			for (j = 0; j < current_vertices; j++) pnu[j] = nu[j];
			delete[] nu; nu = pnu;
			ppts = new double[3 * i];
			for (j = 0; j < 3 * current_vertices; j++) ppts[j] = pts[j];
			delete[] pts; pts = ppts;
			current_vertices = i;
		}

		template<class vc_class>
		void add_memory_vorder(vc_class& vc)
		{
			int i = (current_vertex_order << 1), j, * p1, ** p2;
			if (i > max_vertex_order)
			{
				assert(false);
				// voro_fatal_error("Vertex order memory allocation exceeded absolute maximum", VOROPP_MEMORY_ERROR);
			}
#if VOROPP_VERBOSE >=2
			fprintf(stderr, "Vertex order memory scaled up to %d\n", i);
#endif
			p1 = new int[i];
			for (j = 0; j < current_vertex_order; j++) p1[j] = mem[j]; while (j < i) p1[j++] = 0;
			delete[] mem; mem = p1;
			p2 = new int* [i];
			for (j = 0; j < current_vertex_order; j++) p2[j] = mep[j];
			delete[] mep; mep = p2;
			p1 = new int[i];
			for (j = 0; j < current_vertex_order; j++) p1[j] = mec[j]; while (j < i) p1[j++] = 0;
			delete[] mec; mec = p1;
			vc.n_add_memory_vorder(i);
			current_vertex_order = i;
		}

		void add_memory_ds(int*& stackp)
		{
			current_delete_size <<= 1;
			if (current_delete_size > max_delete_size)
			{
				assert(false);
				// voro_fatal_error("Delete stack 1 memory allocation exceeded absolute maximum", VOROPP_MEMORY_ERROR);
			}
#if VOROPP_VERBOSE >=2
			fprintf(stderr, "Delete stack 1 memory scaled up to %d\n", current_delete_size);
#endif
			int* dsn = new int[current_delete_size], * dsnp = dsn, * dsp = ds;
			while (dsp < stackp) *(dsnp++) = *(dsp++);
			delete[] ds; ds = dsn; stackp = dsnp;
			stacke = ds + current_delete_size;
		}
		void add_memory_ds2(int*& stackp2)
		{
			current_delete2_size <<= 1;
			if (current_delete2_size > max_delete2_size)
			{
				assert(false);
				// voro_fatal_error("Delete stack 2 memory allocation exceeded absolute maximum", VOROPP_MEMORY_ERROR);
			}
#if VOROPP_VERBOSE >=2
			fprintf(stderr, "Delete stack 2 memory scaled up to %d\n", current_delete2_size);
#endif
			int* dsn = new int[current_delete2_size], * dsnp = dsn, * dsp = ds2;
			while (dsp < stackp2) *(dsnp++) = *(dsp++);
			delete[] ds2; ds2 = dsn; stackp2 = dsnp;
			stacke2 = ds2 + current_delete2_size;
		}

		template<class vc_class>
		inline bool collapse_order1(vc_class& vc)
		{
			int i, j, k;
			while (mec[1] > 0) {
				up = 0;
				i = --mec[1];
				j = mep[1][3 * i]; k = mep[1][3 * i + 1];
				i = mep[1][3 * i + 2];
				if (!delete_connection(vc, j, k, false)) return false;
				--p;
				if (up == i) up = 0;
				if (p != i) {
					if (up == p) up = i;
					pts[3 * i] = pts[3 * p];
					pts[3 * i + 1] = pts[3 * p + 1];
					pts[3 * i + 2] = pts[3 * p + 2];
					for (k = 0; k < nu[p]; k++) ed[ed[p][k]][ed[p][nu[p] + k]] = i;
					vc.n_copy_pointer(i, p);
					ed[i] = ed[p];
					nu[i] = nu[p];
					ed[i][nu[i] << 1] = i;
				}
			}
			return true;
		}

		template<class vc_class>
		inline bool collapse_order2(vc_class& vc)
		{
			if (!collapse_order1(vc)) return false;
			int a, b, i, j, k, l;
			while (mec[2] > 0) {

				// Pick a order 2 vertex and read in its edges
				i = --mec[2];
				j = mep[2][5 * i]; k = mep[2][5 * i + 1];
				if (j == k) {
					return false;
				}

				// Scan the edges of j to see if joins k
				for (l = 0; l < nu[j]; l++) {
					if (ed[j][l] == k) break;
				}

				// If j doesn't already join k, join them together.
				// Otherwise delete the connection to the current
				// vertex from j and k.
				a = mep[2][5 * i + 2]; b = mep[2][5 * i + 3]; i = mep[2][5 * i + 4];
				if (l == nu[j]) {
					ed[j][a] = k;
					ed[k][b] = j;
					ed[j][nu[j] + a] = b;
					ed[k][nu[k] + b] = a;
				}
				else {
					if (!delete_connection(vc, j, a, false)) return false;
					if (!delete_connection(vc, k, b, true)) return false;
				}

				// Compact the memory
				--p;
				if (up == i) up = 0;
				if (p != i) {
					if (up == p) up = i;
					pts[3 * i] = pts[3 * p];
					pts[3 * i + 1] = pts[3 * p + 1];
					pts[3 * i + 2] = pts[3 * p + 2];
					for (k = 0; k < nu[p]; k++) ed[ed[p][k]][ed[p][nu[p] + k]] = i;
					vc.n_copy_pointer(i, p);
					ed[i] = ed[p];
					nu[i] = nu[p];
					ed[i][nu[i] << 1] = i;
				}

				// Collapse any order 1 vertices if they were created
				if (!collapse_order1(vc)) return false;
			}
			return true;
		}

		template<class vc_class>
		inline bool delete_connection(vc_class& vc, int j, int k, bool hand)
		{
			int q = hand ? k : cycle_up(k, j);
			int i = nu[j] - 1, l, * edp, * edd, m;
			if (i < 1) {
				return false;
			}
			if (mec[i] == mem[i]) add_memory(vc, i, ds2);
			vc.n_set_aux1(i);
			for (l = 0; l < q; l++) vc.n_copy_aux1(j, l);
			while (l < i) {
				vc.n_copy_aux1_shift(j, l);
				l++;
			}
			edp = mep[i] + ((i << 1) + 1) * mec[i]++;
			edp[i << 1] = j;
			for (l = 0; l < k; l++) {
				edp[l] = ed[j][l];
				edp[l + i] = ed[j][l + nu[j]];
			}
			while (l < i) {
				m = ed[j][l + 1];
				edp[l] = m;
				k = ed[j][l + nu[j] + 1];
				edp[l + i] = k;
				ed[m][nu[m] + k]--;
				l++;
			}

			edd = mep[nu[j]] + ((nu[j] << 1) + 1) * --mec[nu[j]];
			for (l = 0; l <= (nu[j] << 1); l++) ed[j][l] = edd[l];
			vc.n_set_aux2_copy(j, nu[j]);
			vc.n_set_to_aux2(edd[nu[j] << 1]);
			vc.n_set_to_aux1(j);
			ed[edd[nu[j] << 1]] = edd;
			ed[j] = edp;
			nu[j] = i;
			return true;

		}

		template<class vc_class>
		inline bool search_for_outside_edge(vc_class& vc, int& upRef)
		{
			int i, lp, lw, * j(ds2), * stackp2(ds2);
			double l;
			*(stackp2++) = upRef;
			while (j < stackp2) {
				upRef = *(j++);
				for (i = 0; i < nu[upRef]; i++) {
					lp = ed[upRef][i];
					lw = m_test(lp, l);
					if (lw == -1) return true;
					else if (lw == 0) add_to_stack(vc, lp, stackp2);
				}
			}
			return false;
		}

		template<class vc_class>
		inline void add_to_stack(vc_class& vc, int lp, int*& stackp2)
		{
			for (int* k(ds2); k < stackp2; k++) if (*k == lp) return;
			if (stackp2 == stacke2) add_memory_ds2(stackp2);
			*(stackp2++) = lp;
		}

		inline bool plane_intersects_track(double x, double y, double z, double rsq, double g)
		{
			int count = 0, ls, usOuter, tp;
			double t;

			// The test point is outside of the cutting space
			for (usOuter = 0; usOuter < nu[up]; usOuter++) {
				tp = ed[up][usOuter];
				t = x * pts[3 * tp] + y * pts[3 * tp + 1] + z * pts[3 * tp + 2];
				if (t > g) {
					ls = ed[up][nu[up] + usOuter];
					up = tp;
					while (t < rsq) {
						if (++count >= p) {
#if VOROPP_VERBOSE >=1
							fputs("Bailed out of convex calculation", stderr);
#endif
							for (tp = 0; tp < p; tp++) if (x * pts[3 * tp] + y * pts[3 * tp + 1] + z * pts[3 * tp + 2] > rsq) return true;
							return false;
						}

						// Test all the neighbors of the current point
						// and find the one which is closest to the
						// plane
						int usInner;
						for (usInner = 0; usInner < ls; usInner++) {
							tp = ed[up][usInner];
							g = x * pts[3 * tp] + y * pts[3 * tp + 1] + z * pts[3 * tp + 2];
							if (g > t) break;
						}
						if (usInner == ls) {
							usInner++;
							while (usInner < nu[up]) {
								tp = ed[up][usInner];
								g = x * pts[3 * tp] + y * pts[3 * tp + 1] + z * pts[3 * tp + 2];
								if (g > t) break;
								usInner++;
							}
							if (usInner == nu[up]) return false;
						}
						ls = ed[up][nu[up] + usInner]; up = tp; t = g;
					}
					return true;
				}
			}
			return false;
		}

		inline void normals_search(std::vector<double>& v, int i, int j, int k)
		{
			ed[i][j] = -1 - k;
			int l = cycle_up(ed[i][nu[i] + j], k), m;
			double ux, uy, uz, vx, vy, vz, wx, wy, wz, wmag;
			do {
				m = ed[k][l]; ed[k][l] = -1 - m;
				ux = pts[3 * m] - pts[3 * k];
				uy = pts[3 * m + 1] - pts[3 * k + 1];
				uz = pts[3 * m + 2] - pts[3 * k + 2];

				// Test to see if the length of this edge is above the tolerance
				if (ux * ux + uy * uy + uz * uz > tolerance_sq) {
					while (m != i) {
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m; m = ed[k][l]; ed[k][l] = -1 - m;
						vx = pts[3 * m] - pts[3 * k];
						vy = pts[3 * m + 1] - pts[3 * k + 1];
						vz = pts[3 * m + 2] - pts[3 * k + 2];

						// Construct the vector product of this edge with
						// the previous one
						wx = uz * vy - uy * vz;
						wy = ux * vz - uz * vx;
						wz = uy * vx - ux * vy;
						wmag = wx * wx + wy * wy + wz * wz;

						// Test to see if this vector product of the
						// two edges is above the tolerance
						if (wmag > tolerance_sq) {

							// Construct the normal vector and print it
							wmag = 1 / sqrt(wmag);
							v.push_back(wx * wmag);
							v.push_back(wy * wmag);
							v.push_back(wz * wmag);

							// Mark all of the remaining edges of this
							// face and exit
							while (m != i) {
								l = cycle_up(ed[k][nu[k] + l], m);
								k = m; m = ed[k][l]; ed[k][l] = -1 - m;
							}
							return;
						}
					}
					v.push_back(0);
					v.push_back(0);
					v.push_back(0);
					return;
				}
				l = cycle_up(ed[k][nu[k] + l], m);
				k = m;
			} while (k != i);
			v.push_back(0);
			v.push_back(0);
			v.push_back(0);
		}

		inline bool search_edge(int l, int& m, int& k)
		{
			for (m = 0; m < nu[l]; m++) {
				k = ed[l][m];
				if (k >= 0) return true;
			}
			return false;
		}

		inline int m_test(int n, double& ans)
		{
			double* pp = pts + n + (size_t(n) << 1);
			ans = *(pp++) * px;
			ans += *(pp++) * py;
			ans += *pp * pz - prsq;
			if (ans < -tolerance2) {
				return -1;
			}
			else if (ans > tolerance2) {
				return 1;
			}
			return check_marginal(n, ans);
		}

		int check_marginal(int n, double& ans)
		{
			int i;
			for (i = 0; i < n_marg; i += 2) if (marg[i] == n) return marg[i + 1];
			if (n_marg == current_marginal) {
				current_marginal <<= 1;
				if (current_marginal > max_marginal)
				{
					assert(false);
					// voro_fatal_error("Marginal case buffer allocation exceeded absolute maximum", VOROPP_MEMORY_ERROR);
				}
#if VOROPP_VERBOSE >=2
				fprintf(stderr, "Marginal cases buffer scaled up to %d\n", i);
#endif
				int* pmarg = new int[current_marginal];
				for (int j = 0; j < n_marg; j++) pmarg[j] = marg[j];
				delete[] marg;
				marg = pmarg;
			}
			marg[n_marg++] = n;
			marg[n_marg++] = ans > tolerance ? 1 : (ans < -tolerance ? -1 : 0);
			return marg[n_marg - 1];
		}
		friend class voronoicell;
		friend class voronoicell_neighbor;
	};

	/* \brief Extension of the voronoicell_base class to represent a Voronoi
	 * cell without neighbor information.
	 *
	 * This class is an extension of the voronoicell_base class, in cases when
	 * is not necessary to track the IDs of neighboring particles associated
	 * with each face of the Voronoi cell. */
	class voronoicell : public voronoicell_base {
	public:
		using voronoicell_base::nplane;
		/* Copies the information from another voronoicell class into
		 * this class, extending memory allocation if necessary.
		 * \param[in] c the class to copy. */
		inline void operator=(voronoicell& c) {
			voronoicell_base* vb((voronoicell_base*)&c);
			check_memory_for_copy(*this, vb); copy(vb);
		}
		/* Cuts a Voronoi cell using by the plane corresponding to the
		 * perpendicular bisector of a particle.
		 * \param[in] (x,y,z) the position of the particle.
		 * \param[in] rsq the modulus squared of the vector.
		 * \param[in] p_id the plane ID, ignored for this case where no
		 *                 neighbor tracking is enabled.
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool nplane(double x, double y, double z, double rsq, int p_id) {
			return nplane(*this, x, y, z, rsq, 0);
		}
		/* Cuts a Voronoi cell using by the plane corresponding to the
		 * perpendicular bisector of a particle.
		 * \param[in] (x,y,z) the position of the particle.
		 * \param[in] p_id the plane ID, ignored for this case where no
		 *                 neighbor tracking is enabled.
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool nplane(double x, double y, double z, int p_id) {
			double rsq = x * x + y * y + z * z;
			return nplane(*this, x, y, z, rsq, 0);
		}
		/* Cuts a Voronoi cell using by the plane corresponding to the
		 * perpendicular bisector of a particle.
		 * \param[in] (x,y,z) the position of the particle.
		 * \param[in] rsq the modulus squared of the vector.
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool plane(double x, double y, double z, double rsq) {
			return nplane(*this, x, y, z, rsq, 0);
		}
		/* Cuts a Voronoi cell using by the plane corresponding to the
		 * perpendicular bisector of a particle.
		 * \param[in] (x,y,z) the position of the particle.
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool plane(double x, double y, double z) {
			double rsq = x * x + y * y + z * z;
			return nplane(*this, x, y, z, rsq, 0);
		}
		/* Initializes the Voronoi cell to be rectangular box with the
		 * given dimensions.
		 * \param[in] (xmin,xmax) the minimum and maximum x coordinates.
		 * \param[in] (ymin,ymax) the minimum and maximum y coordinates.
		 * \param[in] (zmin,zmax) the minimum and maximum z coordinates. */
		inline void init(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) {
			init_base(xmin, xmax, ymin, ymax, zmin, zmax);
		}
		/* Initializes the cell to be an octahedron with vertices at
		 * (l,0,0), (-l,0,0), (0,l,0), (0,-l,0), (0,0,l), and (0,0,-l).
		 * \param[in] l a parameter setting the size of the octahedron.
		 */
		inline void init_octahedron(double l) {
			init_octahedron_base(l);
		}
		/* Initializes the cell to be a tetrahedron.
		 * \param[in] (x0,y0,z0) the coordinates of the first vertex.
		 * \param[in] (x1,y1,z1) the coordinates of the second vertex.
		 * \param[in] (x2,y2,z2) the coordinates of the third vertex.
		 * \param[in] (x3,y3,z3) the coordinates of the fourth vertex.
		 */
		inline void init_tetrahedron(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3) {
			init_tetrahedron_base(x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3);
		}

		void get_cell_info(const Maths::TVector3<double>& CellPosition, std::vector<Vector3>& Vertices, std::vector<int>& FaceVertexIndices)
		{
			Vertices.resize(p);
			double* ptsp = pts;
			for (int i = 0; i < p; i++)
			{
				Vertices[i].x = (float)(CellPosition.x + *(ptsp++) * 0.5);
				Vertices[i].y = (float)(CellPosition.y + *(ptsp++) * 0.5);
				Vertices[i].z = (float)(CellPosition.z + *(ptsp++) * 0.5);
			}

			FaceVertexIndices.clear();

			// Vertex indices code (face_vertices)
			int i, j, k, l, m, vp(0), vn;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) { // on a new face
					FaceVertexIndices.push_back(0); // placeholder for face vertex count
					FaceVertexIndices.push_back(i);

					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						FaceVertexIndices.push_back(k);
						m = ed[k][l];
						ed[k][l] = -1 - m;

						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);

					vn = (int)FaceVertexIndices.size();
					FaceVertexIndices[vp] = vn - vp - 1;
					vp = vn;
				}
			}
			reset_edges();
		}
	private:
		inline void n_allocate(int i, int m) {};
		inline void n_add_memory_vertices(int i) {};
		inline void n_add_memory_vorder(int i) {};
		inline void n_set_pointer(int neIndex, int n) {};
		inline void n_copy(int a, int b, int c, int d) {};
		inline void n_set(int a, int b, int c) {};
		inline void n_set_aux1(int k) {};
		inline void n_copy_aux1(int a, int b) {};
		inline void n_copy_aux1_shift(int a, int b) {};
		inline void n_set_aux2_copy(int a, int b) {};
		inline void n_copy_pointer(int a, int b) {};
		inline void n_set_to_aux1(int j) {};
		inline void n_set_to_aux2(int j) {};
		inline void n_allocate_aux1(int i) {};
		inline void n_switch_to_aux1(int i) {};
		inline void n_copy_to_aux1(int i, int m) {};
		inline void n_set_to_aux1_offset(int k, int m) {};
		inline void n_neighbors(std::vector<int>& v) { v.clear(); };
		friend class voronoicell_base;
	};

	/* \brief Extension of the voronoicell_base class to represent a Voronoi cell
	 * with neighbor information.
	 *
	 * This class is an extension of the voronoicell_base class, in cases when the
	 * IDs of neighboring particles associated with each face of the Voronoi cell.
	 * It contains additional data structures mne and ne for storing this
	 * information. */
	class voronoicell_neighbor : public voronoicell_base {
	public:
		using voronoicell_base::nplane;
		/* This two dimensional array holds the neighbor information
		 * associated with each vertex. mne[p] is a one dimensional
		 * array which holds all of the neighbor information for
		 * vertices of order p. */
		int** mne;
		/* This is a two dimensional array that holds the neighbor
		 * information associated with each vertex. ne[i] points to a
		 * one-dimensional array in mne[nu[i]]. ne[i][j] holds the
		 * neighbor information associated with the jth edge of vertex
		 * i. It is set to the ID number of the plane that made the
		 * face that is clockwise from the jth edge. */
		int** ne;
		voronoicell_neighbor()
		{
			int i;
			assert(current_vertex_order >= 4);
			mne = new int* [current_vertex_order];
			ne = new int* [current_vertices];
			for (i = 0; i < 3; i++) mne[i] = new int[init_n_vertices * i];
			mne[3] = new int[init_3_vertices * 3];
			for (i = 4; i < current_vertex_order; i++) mne[i] = new int[init_n_vertices * i];
		}

		virtual ~voronoicell_neighbor()
		{
			for (int i = current_vertex_order - 1; i >= 0; i--) if (mem[i] > 0) delete[] mne[i];
			delete[] mne;
			delete[] ne;
		}

		void operator=(voronoicell& c)
		{
			voronoicell_base* vb = ((voronoicell_base*)&c);
			check_memory_for_copy(*this, vb); copy(vb);
			int i, j;
			for (i = 0; i < c.current_vertex_order; i++) {
				for (j = 0; j < c.mec[i] * i; j++) mne[i][j] = 0;
				for (j = 0; j < c.mec[i]; j++) ne[c.mep[i][(2 * i + 1) * j + 2 * i]] = mne[i] + (j * i);
			}
		}

		void operator=(voronoicell_neighbor& c)
		{
			voronoicell_base* vb = ((voronoicell_base*)&c);
			check_memory_for_copy(*this, vb); copy(vb);
			int i, j;
			for (i = 0; i < c.current_vertex_order; i++) {
				for (j = 0; j < c.mec[i] * i; j++) mne[i][j] = c.mne[i][j];
				for (j = 0; j < c.mec[i]; j++) ne[c.mep[i][(2 * i + 1) * j + 2 * i]] = mne[i] + (j * i);
			}
		}
		/* Cuts the Voronoi cell by a particle whose center is at a
		 * separation of (x,y,z) from the cell center. The value of rsq
		 * should be initially set to \f$x^2+y^2+z^2\f$.
		 * \param[in] (x,y,z) the normal vector to the plane.
		 * \param[in] rsq the distance along this vector of the plane.
		 * \param[in] p_id the plane ID (for neighbor tracking only).
		 * \return False if the plane cut deleted the cell entirely,
		 * true otherwise. */
		inline bool nplane(double x, double y, double z, double rsq, int p_id) {
			return nplane(*this, x, y, z, rsq, p_id);
		}
		/* This routine calculates the modulus squared of the vector
		 * before passing it to the main nplane() routine with full
		 * arguments.
		 * \param[in] (x,y,z) the vector to cut the cell by.
		 * \param[in] p_id the plane ID (for neighbor tracking only).
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool nplane(double x, double y, double z, int p_id) {
			double rsq = x * x + y * y + z * z;
			return nplane(*this, x, y, z, rsq, p_id);
		}
		/* This version of the plane routine just makes up the plane
		 * ID to be zero. It will only be referenced if neighbor
		 * tracking is enabled.
		 * \param[in] (x,y,z) the vector to cut the cell by.
		 * \param[in] rsq the modulus squared of the vector.
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool plane(double x, double y, double z, double rsq) {
			return nplane(*this, x, y, z, rsq, 0);
		}
		/* Cuts a Voronoi cell using the influence of a particle at
		 * (x,y,z), first calculating the modulus squared of this
		 * vector before passing it to the main nplane() routine. Zero
		 * is supplied as the plane ID, which will be ignored unless
		 * neighbor tracking is enabled.
		 * \param[in] (x,y,z) the vector to cut the cell by.
		 * \return False if the plane cut deleted the cell entirely,
		 *         true otherwise. */
		inline bool plane(double x, double y, double z) {
			double rsq = x * x + y * y + z * z;
			return nplane(*this, x, y, z, rsq, 0);
		}
		void init(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
		{
			init_base(xmin, xmax, ymin, ymax, zmin, zmax);
			int* q = mne[3];
			*q = -5; q[1] = -3; q[2] = -1;
			q[3] = -5; q[4] = -2; q[5] = -3;
			q[6] = -5; q[7] = -1; q[8] = -4;
			q[9] = -5; q[10] = -4; q[11] = -2;
			q[12] = -6; q[13] = -1; q[14] = -3;
			q[15] = -6; q[16] = -3; q[17] = -2;
			q[18] = -6; q[19] = -4; q[20] = -1;
			q[21] = -6; q[22] = -2; q[23] = -4;
			*ne = q; ne[1] = q + 3; ne[2] = q + 6; ne[3] = q + 9;
			ne[4] = q + 12; ne[5] = q + 15; ne[6] = q + 18; ne[7] = q + 21;
		}

		void init_octahedron(double l)
		{
			init_octahedron_base(l);
			int* q = mne[4];
			*q = -5; q[1] = -6; q[2] = -7; q[3] = -8;
			q[4] = -1; q[5] = -2; q[6] = -3; q[7] = -4;
			q[8] = -6; q[9] = -5; q[10] = -2; q[11] = -1;
			q[12] = -8; q[13] = -7; q[14] = -4; q[15] = -3;
			q[16] = -5; q[17] = -8; q[18] = -3; q[19] = -2;
			q[20] = -7; q[21] = -6; q[22] = -1; q[23] = -4;
			*ne = q; ne[1] = q + 4; ne[2] = q + 8; ne[3] = q + 12; ne[4] = q + 16; ne[5] = q + 20;
		}

		void init_tetrahedron(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3)
		{
			init_tetrahedron_base(x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3);
			int* q = mne[3];
			*q = -4; q[1] = -3; q[2] = -2;
			q[3] = -3; q[4] = -4; q[5] = -1;
			q[6] = -4; q[7] = -2; q[8] = -1;
			q[9] = -2; q[10] = -3; q[11] = -1;
			*ne = q; ne[1] = q + 3; ne[2] = q + 6; ne[3] = q + 9;
		}

		void check_facets()
		{
			int i, j, k, l, m, q;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					ed[i][j] = -1 - k;
					q = ne[i][j];
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						m = ed[k][l];
						ed[k][l] = -1 - m;
						if (ne[k][l] != q) fprintf(stderr, "Facet error at (%d,%d)=%d, started from (%d,%d)=%d\n", k, l, ne[k][l], i, j, q);
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
				}
			}
			reset_edges();
		}
		virtual void neighbors(std::vector<int>& v)
		{
			v.clear();
			int i, j, k, l, m;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					v.push_back(ne[i][j]);
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						m = ed[k][l];
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
				}
			}
			reset_edges();
		}
		
		void neighborsTArray(std::vector<int>& nbrs, bool excludeBounds)
		{
			nbrs.clear();
			int i, j, k, l, m;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) {
					if (!excludeBounds || ne[i][j] >= 0) {
						nbrs.push_back(ne[i][j]);
					}
					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						m = ed[k][l];
						ed[k][l] = -1 - m;
						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);
				}
			}
			reset_edges();
		}

		void get_cell_info(const Maths::TVector3<double>& CellPosition, std::vector<Vector3>& Vertices, std::vector<int>& FaceVertexIndices, std::vector<int>& Nbrs, std::vector<Vector3>& Normals)
		{
			Vertices.resize(p);
			double* ptsp = pts;
			for (int i = 0; i < p; i++)
			{
				Vertices[i].x = (float)(CellPosition.x + *(ptsp++) * 0.5);
				Vertices[i].y = (float)(CellPosition.y + *(ptsp++) * 0.5);
				Vertices[i].z = (float)(CellPosition.z + *(ptsp++) * 0.5);
			}

			FaceVertexIndices.clear(); Nbrs.clear(); Normals.clear();

			// Vertex indices code (face_vertices)
			int i, j, k, l, m, vp(0), vn;
			for (i = 1; i < p; i++) for (j = 0; j < nu[i]; j++) {
				k = ed[i][j];
				if (k >= 0) { // on a new face
					Nbrs.push_back(ne[i][j]);
					FaceVertexIndices.push_back(0); // placeholder for face vertex count
					FaceVertexIndices.push_back(i);
					Vector3 Normal(0, 0, 0);
					double foundEdgeMag = 0;
					double ux = 0, uy = 0, uz = 0, vx, vy, vz, wx, wy, wz, wmag = 0;

					ed[i][j] = -1 - k;
					l = cycle_up(ed[i][nu[i] + j], k);
					do {
						FaceVertexIndices.push_back(k);
						m = ed[k][l];
						ed[k][l] = -1 - m;

						if (wmag <= tolerance_sq) // Haven't found a Normal yet
						{
							if (foundEdgeMag <= tolerance_sq) // Haven't found a first edge yet
							{
								ux = pts[3 * m] - pts[3 * k];
								uy = pts[3 * m + 1] - pts[3 * k + 1];
								uz = pts[3 * m + 2] - pts[3 * k + 2];
								foundEdgeMag = ux * ux + uy * uy + uz * uz;
							}
							else // Have a first edge; try constructing the Normal from each following edge
							{
								vx = pts[3 * m] - pts[3 * k];
								vy = pts[3 * m + 1] - pts[3 * k + 1];
								vz = pts[3 * m + 2] - pts[3 * k + 2];

								// Construct the vector product of this edge with
								// the previously-found one
								wx = uz * vy - uy * vz;
								wy = ux * vz - uz * vx;
								wz = uy * vx - ux * vy;
								wmag = wx * wx + wy * wy + wz * wz;

								// Test to see if this vector product of the
								// two edges is above the tolerance
								if (wmag > tolerance_sq) {
									// Set the normal vector
									double wmagInv = 1.0 / sqrt(wmag);
									Normal.x = (float)(wx * wmagInv);
									Normal.y = (float)(wy * wmagInv);
									Normal.z = (float)(wz * wmagInv);
								}
							}
						}

						l = cycle_up(ed[k][nu[k] + l], m);
						k = m;
					} while (k != i);

					Normals.push_back(Normal);
					vn = (int)FaceVertexIndices.size();
					FaceVertexIndices[vp] = vn - vp - 1;
					vp = vn;
				}
			}
			reset_edges();
		}

		virtual void print_edges_neighbors(int i)
		{
			if (nu[i] > 0) {
				int j = 0;
				printf("     (");
				while (j < nu[i] - 1) printf("%d,", ne[i][j++]);
				printf("%d)", ne[i][j]);
			}
			else printf("     ()");
		}

	private:
		int* paux1;
		int* paux2;
		inline void n_allocate(int i, int m) { mne[i] = new int[m * i]; }
		inline void n_add_memory_vertices(int i) {
			int** pp = new int* [i];
			assert(current_vertices <= i);
			for (int j = 0; j < current_vertices; j++) pp[j] = ne[j];
			delete[] ne; ne = pp;
		}
		inline void n_add_memory_vorder(int i) {
			int** p2 = new int* [i];
			assert(current_vertex_order <= i);
			for (int j = 0; j < current_vertex_order; j++) p2[j] = mne[j];
			delete[] mne; mne = p2;
		}
		inline void n_set_pointer(int neIndex, int n) {
			ne[neIndex] = mne[n] + n * mec[n];
		}
		inline void n_copy(int a, int b, int c, int d) { ne[a][b] = ne[c][d]; }
		inline void n_set(int a, int b, int c) { ne[a][b] = c; }
		inline void n_set_aux1(int k) { paux1 = mne[k] + k * mec[k]; }
		inline void n_copy_aux1(int a, int b) { paux1[b] = ne[a][b]; }
		inline void n_copy_aux1_shift(int a, int b) { paux1[b] = ne[a][b + 1]; }
		inline void n_set_aux2_copy(int a, int b) {
			paux2 = mne[b] + b * mec[b];
			for (int i = 0; i < b; i++) ne[a][i] = paux2[i];
		}
		inline void n_copy_pointer(int a, int b) { ne[a] = ne[b]; }
		inline void n_set_to_aux1(int j) { ne[j] = paux1; }
		inline void n_set_to_aux2(int j) { ne[j] = paux2; }
		inline void n_allocate_aux1(int i) { paux1 = new int[i * mem[i]]; }
		inline void n_switch_to_aux1(int i) { delete[] mne[i]; mne[i] = paux1; }
		inline void n_copy_to_aux1(int i, int m) { paux1[m] = mne[i][m]; }
		inline void n_set_to_aux1_offset(int k, int m) { ne[k] = paux1 + m; }
		friend class voronoicell_base;
	};

	/* \brief Class containing all of the routines that are specific to computing
	 * the regular Voronoi tessellation.
	 *
	 * The container and container_periodic classes are derived from this class,
	 * and during the Voronoi cell computation, these routines are used to create
	 * the regular Voronoi tessellation. */
	class radius_mono {
	public:
		struct radius_info // intentionally empty; no special radius info needs to be passed in for radius_mono case
		{
		};
	protected:
		/* This is called prior to computing a Voronoi cell for a
		 * given particle to initialize any required constants.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] s the index of the particle within the block. */
		inline void r_init(int ijk, int s, radius_info& radius_out) const {}
		/* Sets a required constant to be used when carrying out a
		 * plane bounds check. */
		inline void r_prime(double rv, radius_info& radius_out) const {}
		/* Carries out a radius bounds check.
		 * \param[in] crs the radius squared to be tested.
		 * \param[in] mrs the current maximum distance to a Voronoi
		 *                vertex multiplied by two.
		 * \return True if particles at this radius could not possibly
		 * cut the cell, false otherwise. */
		inline bool r_ctest(double crs, double mrs, const radius_info& rad_info) const { return crs > mrs; }
		/* Scales a plane displacement during a plane bounds check.
		 * \param[in] lrs the plane displacement.
		 * \return The scaled value. */
		inline double r_cutoff(double lrs, const radius_info& rad_info) const { return lrs; }
		/* Adds the maximum radius squared to a given value.
		 * \param[in] rs the value to consider.
		 * \return The value with the radius squared added. */
		inline double r_max_add(double rs) const { return rs; }
		/* Subtracts the radius squared of a particle from a given
		 * value.
		 * \param[in] rs the value to consider.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return The value with the radius squared subtracted. */
		inline double r_current_sub(double rs, int ijk, int q) const { return rs; }
		/* Scales a plane displacement prior to use in the plane cutting
		 * algorithm.
		 * \param[in] rs the initial plane displacement.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return The scaled plane displacement. */
		inline double r_scale(double rs, int ijk, int q, const radius_info& rad_info) const { return rs; }
		/* Scales a plane displacement prior to use in the plane
		 * cutting algorithm, and also checks if it could possibly cut
		 * the cell.
		 * \param[in,out] rs the plane displacement to be scaled.
		 * \param[in] mrs the current maximum distance to a Voronoi
		 *                vertex multiplied by two.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return True if the cell could possibly cut the cell, false
		 * otherwise. */
		inline bool r_scale_check(double& rs, double mrs, int ijk, int q, const radius_info& rad_info) const { return rs < mrs; }
	};

	/*  \brief Class containing all of the routines that are specific to computing
	 * the radical Voronoi tessellation.
	 *
	 * The container_poly and container_periodic_poly classes are derived from this
	 * class, and during the Voronoi cell computation, these routines are used to
	 * create the radical Voronoi tessellation. */
	class radius_poly {
	public:
		struct radius_info
		{
			double r_rad, r_mul, r_val;
		};
		/* A two-dimensional array holding particle positions and radii. */
		double** ppr;
		/* The current maximum radius of any particle, used to
		 * determine when to cut off the radical Voronoi computation.
		 * */
		double max_radius;
		/* The class constructor sets the maximum particle radius to
		 * be zero. */
		radius_poly() : max_radius(0), ppr(nullptr) {}
	protected:
		/* This is called prior to computing a Voronoi cell for a
		 * given particle to initialize any required constants.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] s the index of the particle within the block. */
		inline void r_init(int ijk, int s, radius_info& radius_out) const {
			radius_out.r_rad = ppr[ijk][4 * s + 3] * ppr[ijk][4 * s + 3];
			radius_out.r_mul = radius_out.r_rad - max_radius * max_radius;
		}
		/* Sets a required constant to be used when carrying out a
		 * plane bounds check. */
		inline void r_prime(double rv, radius_info& radius_out) const {
			radius_out.r_val = 1 + radius_out.r_mul / rv;
		}
		/* Carries out a radius bounds check.
		 * \param[in] crs the radius squared to be tested.
		 * \param[in] mrs the current maximum distance to a Voronoi
		 *                vertex multiplied by two.
		 * \return True if particles at this radius could not possibly
		 * cut the cell, false otherwise. */
		inline bool r_ctest(double crs, double mrs, const radius_info& rad_info) const { return crs + rad_info.r_mul > sqrt(mrs * crs); }
		/* Scales a plane displacement during a plane bounds check.
		 * \param[in] lrs the plane displacement.
		 * \return The scaled value. */
		inline double r_cutoff(double lrs, const radius_info& rad_info) const { return lrs * rad_info.r_val; }
		/* Adds the maximum radius squared to a given value.
		 * \param[in] rs the value to consider.
		 * \return The value with the radius squared added. */
		inline double r_max_add(double rs) const { return rs + max_radius * max_radius; }
		/* Subtracts the radius squared of a particle from a given
		 * value.
		 * \param[in] rs the value to consider.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return The value with the radius squared subtracted. */
		inline double r_current_sub(double rs, int ijk, int q) const {
			return rs - ppr[ijk][4 * q + 3] * ppr[ijk][4 * q + 3];
		}
		/* Scales a plane displacement prior to use in the plane cutting
		 * algorithm.
		 * \param[in] rs the initial plane displacement.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return The scaled plane displacement. */
		inline double r_scale(double rs, int ijk, int q, const radius_info& rad_info) const {
			return rs + rad_info.r_rad - ppr[ijk][4 * q + 3] * ppr[ijk][4 * q + 3];
		}
		/* Scales a plane displacement prior to use in the plane
		 * cutting algorithm, and also checks if it could possibly cut
		 * the cell.
		 * \param[in,out] rs the plane displacement to be scaled.
		 * \param[in] mrs the current maximum distance to a Voronoi
		 *                vertex multiplied by two.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return True if the cell could possibly cut the cell, false
		 * otherwise. */
		inline bool r_scale_check(double& rs, double mrs, int ijk, int q, const radius_info& rad_info) const {
			double trs = rs;
			rs += rad_info.r_rad - ppr[ijk][4 * q + 3] * ppr[ijk][4 * q + 3];
			return rs < sqrt(mrs* trs);
		}
	};

	/* \brief Structure for holding information about a particle.
 *
 * This small structure holds information about a single particle, and is used
 * by several of the routines in the voro_compute template for passing
 * information by reference between functions. */
	struct particle_record {
		/* The index of the block that the particle is within. */
		int ijk;
		/* The number of particle within its block. */
		int l;
		/* The x-index of the block. */
		int di;
		/* The y-index of the block. */
		int dj;
		/* The z-index of the block. */
		int dk;
	};

	/* \brief Template for carrying out Voronoi cell computations. */
	template <class c_class>
	class voro_compute {
	public:
		/* A reference to the container class on which to carry out*/
		const c_class& con;
		/* The size of an internal computational block in the x
		 * direction. */
		const double boxx;
		/* The size of an internal computational block in the y
		 * direction. */
		const double boxy;
		/* The size of an internal computational block in the z
		 * direction. */
		const double boxz;
		/* The inverse box length in the x direction, set to
		 * nx/(bx-ax). */
		const double xsp;
		/* The inverse box length in the y direction, set to
		 * ny/(by-ay). */
		const double ysp;
		/* The inverse box length in the z direction, set to
		 * nz/(bz-az). */
		const double zsp;
		/* The number of boxes in the x direction for the searching mask. */
		const int hx;
		/* The number of boxes in the y direction for the searching mask. */
		const int hy;
		/* The number of boxes in the z direction for the searching mask. */
		const int hz;
		/* A constant, set to the value of hx multiplied by hy, which
		 * is used in the routines which step through mask boxes in
		 * sequence. */
		const int hxy;
		/* A constant, set to the value of hx*hy*hz, which is used in
		 * the routines which step through mask boxes in sequence. */
		const int hxyz;
		/* The number of floating point entries to store for each
		 * particle. */
		const int ps;

		// TODO: these are all things we could just access via the container ref con, above? no reason for them to live on this class?
		/* This array holds the numerical IDs of each particle in each
		 * computational box. */
		int** id;
		/* A two dimensional array holding particle positions. For the
		 * derived container_poly class, this also holds particle
		 * radii. */
		double** p;
		/* An array holding the number of particles within each
		 * computational box of the container. */
		int* co;

		voro_compute(c_class& con_, int hx_, int hy_, int hz_)
		:	con(con_), boxx(con_.boxx), boxy(con_.boxy), boxz(con_.boxz),
			xsp(con_.xsp), ysp(con_.ysp), zsp(con_.zsp),
			hx(hx_), hy(hy_), hz(hz_), hxy(hx_* hy_), hxyz(hxy* hz_), ps(con_.ps),
			id(con_.id), p(con_.p), co(con_.co), bxsq(boxx* boxx + boxy * boxy + boxz * boxz),
			mv(0), qu_size(3 * (3 + hxy + hz * (hx + hy))), wl(con_.wl), mrad(con_.mrad),
			mask(new unsigned int[hxyz]), qu(new int[qu_size]), qu_l(qu + qu_size) {
			reset_mask();
		}
		/* The class destructor frees the dynamically allocated memory
		 * for the mask and queue. */
		~voro_compute() {
			delete[] qu;
			delete[] mask;
		}
		voro_compute(const voro_compute<c_class>& Other) = delete;
		voro_compute<c_class>& operator=(const voro_compute<c_class>& Other) = delete;
		voro_compute(voro_compute<c_class>&& Other) noexcept : con(Other.con), boxx(Other.boxx), boxy(Other.boxy), boxz(Other.boxz),
			xsp(Other.xsp), ysp(Other.ysp), zsp(Other.zsp),
			hx(Other.hx), hy(Other.hy), hz(Other.hz), hxy(Other.hxy), hxyz(Other.hxyz), ps(Other.ps),
			id(Other.id), p(Other.p), co(Other.co), bxsq(Other.bxsq),
			mv(Other.mv), qu_size(Other.qu_size), wl(Other.wl), mrad(Other.mrad),
			mask(Other.mask), qu(Other.qu), qu_l(Other.qu_l)
		{
			Other.qu = nullptr;
			Other.mask = nullptr;
		}
		// Note: Move assignment not implemented due to const members on this class (and it has not been needed yet)
		voro_compute<c_class>& operator=(voro_compute<c_class>&& Other) = delete;

		template<class v_cell>
		bool compute_cell(v_cell& c, int ijk, int s, int ci, int cj, int ck)
		{
			static const int count_list[8] = { 7,11,15,19,26,35,45,59 }, * count_e = count_list + 8;
			double x, y, z, x1, y1, z1, qx = 0, qy = 0, qz = 0;
			double xlo, ylo, zlo, xhi, yhi, zhi, x2, y2, z2, rs;
			int i, j, k, di, dj, dk, ei, ej, ek, f, g, l, disp;
			double fx, fy, fz, gxs, gys, gzs, * radp;
			unsigned int q, * e, * mijk;

			if (!con.initialize_voronoicell(c, ijk, s, ci, cj, ck, i, j, k, x, y, z, disp)) return false;
			typename c_class::radius_info radi;
			con.r_init(ijk, s, radi);

			// Initialize the Voronoi cell to fill the entire container
			double crs, mrs;

			int next_count = 3, * count_p = (const_cast<int*> (count_list));

			// Test all particles in the particle's local region first
			for (l = 0; l < s; l++) {
				x1 = p[ijk][ps * l] - x;
				y1 = p[ijk][ps * l + 1] - y;
				z1 = p[ijk][ps * l + 2] - z;
				rs = con.r_scale(x1 * x1 + y1 * y1 + z1 * z1, ijk, l, radi);
				if (!c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
			}
			l++;
			while (l < co[ijk]) {
				x1 = p[ijk][ps * l] - x;
				y1 = p[ijk][ps * l + 1] - y;
				z1 = p[ijk][ps * l + 2] - z;
				rs = con.r_scale(x1 * x1 + y1 * y1 + z1 * z1, ijk, l, radi);
				if (!c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
				l++;
			}

			// Now compute the maximum distance squared from the cell center to a
			// vertex. This is used to cut off the calculation since we only need
			// to test out to twice this range.
			mrs = c.max_radius_squared();

			// Now compute the fractional position of the particle within its
			// region and store it in (fx,fy,fz). We use this to compute an index
			// (di,dj,dk) of which subregion the particle is within.
			unsigned int m1, m2;
			con.frac_pos(x, y, z, ci, cj, ck, fx, fy, fz);
			di = int(fx * xsp * wl_fgrid); dj = int(fy * ysp * wl_fgrid); dk = int(fz * zsp * wl_fgrid);

			// The indices (di,dj,dk) tell us which worklist to use, to test the
			// blocks in the optimal order. But we only store worklists for the
			// eighth of the region where di, dj, and dk are all less than half the
			// full grid. The rest of the cases are handled by symmetry. In this
			// section, we detect for these cases, by reflecting high values of di,
			// dj, and dk. For these cases, a mask is constructed in m1 and m2
			// which is used to flip the worklist information when it is loaded.
			if (di >= wl_hgrid) {
				gxs = fx;
				m1 = 127 + (3 << 21); m2 = 1 + (1 << 21); di = wl_fgrid - 1 - di; if (di < 0) di = 0;
			}
			else { m1 = m2 = 0; gxs = boxx - fx; }
			if (dj >= wl_hgrid) {
				gys = fy;
				m1 |= (127 << 7) + (3 << 24); m2 |= (1 << 7) + (1 << 24); dj = wl_fgrid - 1 - dj; if (dj < 0) dj = 0;
			}
			else gys = boxy - fy;
			if (dk >= wl_hgrid) {
				gzs = fz;
				m1 |= (127 << 14) + (3 << 27); m2 |= (1 << 14) + (1 << 27); dk = wl_fgrid - 1 - dk; if (dk < 0) dk = 0;
			}
			else gzs = boxz - fz;
			gxs *= gxs; gys *= gys; gzs *= gzs;

			// Now compute which worklist we are going to use, and set radp and e to
			// point at the right offsets
			ijk = di + wl_hgrid * (dj + wl_hgrid * dk);
			radp = mrad + ijk * wl_seq_length;
			e = (const_cast<unsigned int*> (wl)) + ijk * wl_seq_length;

			// Read in how many items in the worklist can be tested without having to
			// worry about writing to the mask
			f = e[0]; g = 0;
			do {

				// At the intervals specified by count_list, we recompute the
				// maximum radius squared
				if (g == next_count) {
					mrs = c.max_radius_squared();
					if (count_p != count_e) next_count = *(count_p++);
				}

				// If mrs is less than the minimum distance to any untested
				// block, then we are done
				if (con.r_ctest(radp[g], mrs, radi)) return true;
				g++;

				// Load in a block off the worklist, permute it with the
				// symmetry mask, and decode its position. These are all
				// integer bit operations so they should run very fast.
				q = e[g]; q ^= m1; q += m2;
				di = q & 127; di -= 64;
				dj = (q >> 7) & 127; dj -= 64;
				dk = (q >> 14) & 127; dk -= 64;

				// Check that the worklist position is in range
				ei = di + i; if (ei < 0 || ei >= hx) continue;
				ej = dj + j; if (ej < 0 || ej >= hy) continue;
				ek = dk + k; if (ek < 0 || ek >= hz) continue;

				// Call the compute_min_max_radius() function. This returns
				// true if the minimum distance to the block is bigger than the
				// current mrs, in which case we skip this block and move on.
				// Otherwise, it computes the maximum distance to the block and
				// returns it in crs.
				if (compute_min_max_radius(di, dj, dk, fx, fy, fz, gxs, gys, gzs, crs, mrs, radi)) continue;

				// Now compute which region we are going to loop over, adding a
				// displacement for the periodic cases
				ijk = con.region_index(ci, cj, ck, ei, ej, ek, qx, qy, qz, disp);

				// If mrs is bigger than the maximum distance to the block,
				// then we have to test all particles in the block for
				// intersections. Otherwise, we do additional checks and skip
				// those particles which can't possibly intersect the block.
				if (co[ijk] > 0) {
					l = 0; x2 = x - qx; y2 = y - qy; z2 = z - qz;
					if (!con.r_ctest(crs, mrs, radi)) {
						do {
							x1 = p[ijk][ps * l] - x2;
							y1 = p[ijk][ps * l + 1] - y2;
							z1 = p[ijk][ps * l + 2] - z2;
							rs = con.r_scale(x1 * x1 + y1 * y1 + z1 * z1, ijk, l, radi);
							if (!c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
							l++;
						} while (l < co[ijk]);
					}
					else {
						do {
							x1 = p[ijk][ps * l] - x2;
							y1 = p[ijk][ps * l + 1] - y2;
							z1 = p[ijk][ps * l + 2] - z2;
							rs = x1 * x1 + y1 * y1 + z1 * z1;
							if (con.r_scale_check(rs, mrs, ijk, l, radi) && !c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
							l++;
						} while (l < co[ijk]);
					}
				}
			} while (g < f);

			// If we reach here, we were unable to compute the entire cell using
			// the first part of the worklist. This section of the algorithm
			// continues the worklist, but it now starts preparing the mask that we
			// need if we end up going block by block. We do the same as before,
			// but we put a mark down on the mask for every block that's tested.
			// The worklist also contains information about which neighbors of each
			// block are not also on the worklist, and we start storing those
			// points in a list in case we have to go block by block. Update the
			// mask counter, and if it wraps around then reset the whole mask; that
			// will only happen once every 2^32 tries.
			mv++;
			if (mv == 0) { reset_mask(); mv = 1; }

			// Set the queue pointers
			int* qu_s = qu, * qu_e = qu;

			while (g < wl_seq_length - 1) {

				// At the intervals specified by count_list, we recompute the
				// maximum radius squared
				if (g == next_count) {
					mrs = c.max_radius_squared();
					if (count_p != count_e) next_count = *(count_p++);
				}

				// If mrs is less than the minimum distance to any untested
				// block, then we are done
				if (con.r_ctest(radp[g], mrs, radi)) return true;
				g++;

				// Load in a block off the worklist, permute it with the
				// symmetry mask, and decode its position. These are all
				// integer bit operations so they should run very fast.
				q = e[g]; q ^= m1; q += m2;
				di = q & 127; di -= 64;
				dj = (q >> 7) & 127; dj -= 64;
				dk = (q >> 14) & 127; dk -= 64;

				// Compute the position in the mask of the current block. If
				// this lies outside the mask, then skip it. Otherwise, mark
				// it.
				ei = di + i; if (ei < 0 || ei >= hx) continue;
				ej = dj + j; if (ej < 0 || ej >= hy) continue;
				ek = dk + k; if (ek < 0 || ek >= hz) continue;
				mijk = mask + ei + hx * (ej + hy * ek);
				*mijk = mv;

				// Call the compute_min_max_radius() function. This returns
				// true if the minimum distance to the block is bigger than the
				// current mrs, in which case we skip this block and move on.
				// Otherwise, it computes the maximum distance to the block and
				// returns it in crs.
				if (compute_min_max_radius(di, dj, dk, fx, fy, fz, gxs, gys, gzs, crs, mrs, radi)) continue;

				// Now compute which region we are going to loop over, adding a
				// displacement for the periodic cases
				ijk = con.region_index(ci, cj, ck, ei, ej, ek, qx, qy, qz, disp);

				// If mrs is bigger than the maximum distance to the block,
				// then we have to test all particles in the block for
				// intersections. Otherwise, we do additional checks and skip
				// those particles which can't possibly intersect the block.
				if (co[ijk] > 0) {
					l = 0; x2 = x - qx; y2 = y - qy; z2 = z - qz;
					if (!con.r_ctest(crs, mrs, radi)) {
						do {
							x1 = p[ijk][ps * l] - x2;
							y1 = p[ijk][ps * l + 1] - y2;
							z1 = p[ijk][ps * l + 2] - z2;
							rs = con.r_scale(x1 * x1 + y1 * y1 + z1 * z1, ijk, l, radi);
							if (!c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
							l++;
						} while (l < co[ijk]);
					}
					else {
						do {
							x1 = p[ijk][ps * l] - x2;
							y1 = p[ijk][ps * l + 1] - y2;
							z1 = p[ijk][ps * l + 2] - z2;
							rs = x1 * x1 + y1 * y1 + z1 * z1;
							if (con.r_scale_check(rs, mrs, ijk, l, radi) && !c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
							l++;
						} while (l < co[ijk]);
					}
				}

				// If there might not be enough memory on the list for these
				// additions, then add more
				if (qu_e > qu_l - 18) add_list_memory(qu_s, qu_e);

				// Test the parts of the worklist element which tell us what
				// neighbors of this block are not on the worklist. Store them
				// on the block list, and mark the mask.
				scan_bits_mask_add(q, mijk, ei, ej, ek, qu_e);
			}

			// Do a check to see if we've reached the radius cutoff
			if (con.r_ctest(radp[g], mrs, radi)) return true;

			// We were unable to completely compute the cell based on the blocks in
			// the worklist, so now we have to go block by block, reading in items
			// off the list
			while (qu_s != qu_e) {

				// If we reached the end of the list memory loop back to the
				// start
				if (qu_s == qu_l) qu_s = qu;

				// Read in a block off the list, and compute the upper and lower
				// coordinates in each of the three dimensions
				ei = *(qu_s++); ej = *(qu_s++); ek = *(qu_s++);
				xlo = (ei - i) * boxx - fx; xhi = xlo + boxx;
				ylo = (ej - j) * boxy - fy; yhi = ylo + boxy;
				zlo = (ek - k) * boxz - fz; zhi = zlo + boxz;

				// Carry out plane tests to see if any particle in this block
				// could possibly intersect the cell
				if (ei > i) {
					if (ej > j) {
						if (ek > k) { if (corner_test(c, xlo, ylo, zlo, xhi, yhi, zhi, radi)) continue; }
						else if (ek < k) { if (corner_test(c, xlo, ylo, zhi, xhi, yhi, zlo, radi)) continue; }
						else { if (edge_z_test(c, xlo, ylo, zlo, xhi, yhi, zhi, radi)) continue; }
					}
					else if (ej < j) {
						if (ek > k) { if (corner_test(c, xlo, yhi, zlo, xhi, ylo, zhi, radi)) continue; }
						else if (ek < k) { if (corner_test(c, xlo, yhi, zhi, xhi, ylo, zlo, radi)) continue; }
						else { if (edge_z_test(c, xlo, yhi, zlo, xhi, ylo, zhi, radi)) continue; }
					}
					else {
						if (ek > k) { if (edge_y_test(c, xlo, ylo, zlo, xhi, yhi, zhi, radi)) continue; }
						else if (ek < k) { if (edge_y_test(c, xlo, ylo, zhi, xhi, yhi, zlo, radi)) continue; }
						else { if (face_x_test(c, xlo, ylo, zlo, yhi, zhi, radi)) continue; }
					}
				}
				else if (ei < i) {
					if (ej > j) {
						if (ek > k) { if (corner_test(c, xhi, ylo, zlo, xlo, yhi, zhi, radi)) continue; }
						else if (ek < k) { if (corner_test(c, xhi, ylo, zhi, xlo, yhi, zlo, radi)) continue; }
						else { if (edge_z_test(c, xhi, ylo, zlo, xlo, yhi, zhi, radi)) continue; }
					}
					else if (ej < j) {
						if (ek > k) { if (corner_test(c, xhi, yhi, zlo, xlo, ylo, zhi, radi)) continue; }
						else if (ek < k) { if (corner_test(c, xhi, yhi, zhi, xlo, ylo, zlo, radi)) continue; }
						else { if (edge_z_test(c, xhi, yhi, zlo, xlo, ylo, zhi, radi)) continue; }
					}
					else {
						if (ek > k) { if (edge_y_test(c, xhi, ylo, zlo, xlo, yhi, zhi, radi)) continue; }
						else if (ek < k) { if (edge_y_test(c, xhi, ylo, zhi, xlo, yhi, zlo, radi)) continue; }
						else { if (face_x_test(c, xhi, ylo, zlo, yhi, zhi, radi)) continue; }
					}
				}
				else {
					if (ej > j) {
						if (ek > k) { if (edge_x_test(c, xlo, ylo, zlo, xhi, yhi, zhi, radi)) continue; }
						else if (ek < k) { if (edge_x_test(c, xlo, ylo, zhi, xhi, yhi, zlo, radi)) continue; }
						else { if (face_y_test(c, xlo, ylo, zlo, xhi, zhi, radi)) continue; }
					}
					else if (ej < j) {
						if (ek > k) { if (edge_x_test(c, xlo, yhi, zlo, xhi, ylo, zhi, radi)) continue; }
						else if (ek < k) { if (edge_x_test(c, xlo, yhi, zhi, xhi, ylo, zlo, radi)) continue; }
						else { if (face_y_test(c, xlo, yhi, zlo, xhi, zhi, radi)) continue; }
					}
					else {
						if (ek > k) { if (face_z_test(c, xlo, ylo, zlo, xhi, yhi, radi)) continue; }
						else if (ek < k) { if (face_z_test(c, xlo, ylo, zhi, xhi, yhi, radi)) continue; }
						else assert(false);
					}
				}

				// Now compute the region that we are going to test over, and
				// set a displacement vector for the periodic cases
				ijk = con.region_index(ci, cj, ck, ei, ej, ek, qx, qy, qz, disp);

				// Loop over all the elements in the block to test for cuts. It
				// would be possible to exclude some of these cases by testing
				// against mrs, but this will probably not save time.
				if (co[ijk] > 0) {
					l = 0; x2 = x - qx; y2 = y - qy; z2 = z - qz;
					do {
						x1 = p[ijk][ps * l] - x2;
						y1 = p[ijk][ps * l + 1] - y2;
						z1 = p[ijk][ps * l + 2] - z2;
						rs = con.r_scale(x1 * x1 + y1 * y1 + z1 * z1, ijk, l, radi);
						if (!c.nplane(x1, y1, z1, rs, id[ijk][l])) return false;
						l++;
					} while (l < co[ijk]);
				}

				// If there's not much memory on the block list then add more
				if ((qu_s <= qu_e ? (qu_l - qu_e) + (qu_s - qu) : qu_s - qu_e) < 18) add_list_memory(qu_s, qu_e);

				// Test the neighbors of the current block, and add them to the
				// block list if they haven't already been tested
				add_to_mask(ei, ej, ek, qu_e);
			}

			return true;

		}

		void find_voronoi_cell(double x, double y, double z, int ci, int cj, int ck, int ijk, particle_record& w, double& mrs)
		{
			double qx = 0, qy = 0, qz = 0, rs;
			int i, j, k, di, dj, dk, ei, ej, ek, f, g, disp;
			double fx, fy, fz, mxs, mys, mzs, * radp;
			unsigned int q, * e, * mijk;

			// Init setup for parameters to return
			w.ijk = -1; mrs = large_number;

			con.initialize_search(ci, cj, ck, ijk, i, j, k, disp);

			// Test all particles in the particle's local region first
			scan_all(ijk, x, y, z, 0, 0, 0, w, mrs);

			// Now compute the fractional position of the particle within its
			// region and store it in (fx,fy,fz). We use this to compute an index
			// (di,dj,dk) of which subregion the particle is within.
			unsigned int m1, m2;
			con.frac_pos(x, y, z, ci, cj, ck, fx, fy, fz);
			di = int(fx * xsp * wl_fgrid); dj = int(fy * ysp * wl_fgrid); dk = int(fz * zsp * wl_fgrid);

			// The indices (di,dj,dk) tell us which worklist to use, to test the
			// blocks in the optimal order. But we only store worklists for the
			// eighth of the region where di, dj, and dk are all less than half the
			// full grid. The rest of the cases are handled by symmetry. In this
			// section, we detect for these cases, by reflecting high values of di,
			// dj, and dk. For these cases, a mask is constructed in m1 and m2
			// which is used to flip the worklist information when it is loaded.
			if (di >= wl_hgrid) {
				mxs = boxx - fx;
				m1 = 127 + (3 << 21); m2 = 1 + (1 << 21); di = wl_fgrid - 1 - di; if (di < 0) di = 0;
			}
			else { m1 = m2 = 0; mxs = fx; }
			if (dj >= wl_hgrid) {
				mys = boxy - fy;
				m1 |= (127 << 7) + (3 << 24); m2 |= (1 << 7) + (1 << 24); dj = wl_fgrid - 1 - dj; if (dj < 0) dj = 0;
			}
			else mys = fy;
			if (dk >= wl_hgrid) {
				mzs = boxz - fz;
				m1 |= (127 << 14) + (3 << 27); m2 |= (1 << 14) + (1 << 27); dk = wl_fgrid - 1 - dk; if (dk < 0) dk = 0;
			}
			else mzs = fz;

			// Do a quick test to account for the case when the minimum radius is
			// small enought that no other blocks need to be considered
			rs = con.r_max_add(mrs);
			if (mxs * mxs > rs && mys * mys > rs && mzs * mzs > rs) return;

			// Now compute which worklist we are going to use, and set radp and e to
			// point at the right offsets
			ijk = di + wl_hgrid * (dj + wl_hgrid * dk);
			radp = mrad + ijk * wl_seq_length;
			e = (const_cast<unsigned int*> (wl)) + ijk * wl_seq_length;

			// Read in how many items in the worklist can be tested without having to
			// worry about writing to the mask
			f = e[0]; g = 0;
			do {

				// If mrs is less than the minimum distance to any untested
				// block, then we are done
				if (con.r_max_add(mrs) < radp[g]) return;
				g++;

				// Load in a block off the worklist, permute it with the
				// symmetry mask, and decode its position. These are all
				// integer bit operations so they should run very fast.
				q = e[g]; q ^= m1; q += m2;
				di = q & 127; di -= 64;
				dj = (q >> 7) & 127; dj -= 64;
				dk = (q >> 14) & 127; dk -= 64;

				// Check that the worklist position is in range
				ei = di + i; if (ei < 0 || ei >= hx) continue;
				ej = dj + j; if (ej < 0 || ej >= hy) continue;
				ek = dk + k; if (ek < 0 || ek >= hz) continue;

				// Call the compute_min_max_radius() function. This returns
				// true if the minimum distance to the block is bigger than the
				// current mrs, in which case we skip this block and move on.
				// Otherwise, it computes the maximum distance to the block and
				// returns it in crs.
				if (compute_min_radius(di, dj, dk, fx, fy, fz, mrs)) continue;

				// Now compute which region we are going to loop over, adding a
				// displacement for the periodic cases
				ijk = con.region_index(ci, cj, ck, ei, ej, ek, qx, qy, qz, disp);

				// If mrs is bigger than the maximum distance to the block,
				// then we have to test all particles in the block for
				// intersections. Otherwise, we do additional checks and skip
				// those particles which can't possibly intersect the block.
				scan_all(ijk, x - qx, y - qy, z - qz, di, dj, dk, w, mrs);
			} while (g < f);

			// Update mask value and initialize queue
			mv++;
			if (mv == 0) { reset_mask(); mv = 1; }
			int* qu_s = qu, * qu_e = qu;

			while (g < wl_seq_length - 1) {

				// If mrs is less than the minimum distance to any untested
				// block, then we are done
				if (con.r_max_add(mrs) < radp[g]) return;
				g++;

				// Load in a block off the worklist, permute it with the
				// symmetry mask, and decode its position. These are all
				// integer bit operations so they should run very fast.
				q = e[g]; q ^= m1; q += m2;
				di = q & 127; di -= 64;
				dj = (q >> 7) & 127; dj -= 64;
				dk = (q >> 14) & 127; dk -= 64;

				// Compute the position in the mask of the current block. If
				// this lies outside the mask, then skip it. Otherwise, mark
				// it.
				ei = di + i; if (ei < 0 || ei >= hx) continue;
				ej = dj + j; if (ej < 0 || ej >= hy) continue;
				ek = dk + k; if (ek < 0 || ek >= hz) continue;
				mijk = mask + ei + hx * (ej + hy * ek);
				*mijk = mv;

				// Skip this block if it is further away than the current
				// minimum radius
				if (compute_min_radius(di, dj, dk, fx, fy, fz, mrs)) continue;

				// Now compute which region we are going to loop over, adding a
				// displacement for the periodic cases
				ijk = con.region_index(ci, cj, ck, ei, ej, ek, qx, qy, qz, disp);
				scan_all(ijk, x - qx, y - qy, z - qz, di, dj, dk, w, mrs);

				if (qu_e > qu_l - 18) add_list_memory(qu_s, qu_e);
				scan_bits_mask_add(q, mijk, ei, ej, ek, qu_e);
			}

			// Do a check to see if we've reached the radius cutoff
			if (con.r_max_add(mrs) < radp[g]) return;

			// We were unable to completely compute the cell based on the blocks in
			// the worklist, so now we have to go block by block, reading in items
			// off the list
			while (qu_s != qu_e) {

				// Read the next entry of the queue
				if (qu_s == qu_l) qu_s = qu;
				ei = *(qu_s++); ej = *(qu_s++); ek = *(qu_s++);
				di = ei - i; dj = ej - j; dk = ek - k;
				if (compute_min_radius(di, dj, dk, fx, fy, fz, mrs)) continue;

				ijk = con.region_index(ci, cj, ck, ei, ej, ek, qx, qy, qz, disp);
				scan_all(ijk, x - qx, y - qy, z - qz, di, dj, dk, w, mrs);

				// Test the neighbors of the current block, and add them to the
				// block list if they haven't already been tested
				if ((qu_s <= qu_e ? (qu_l - qu_e) + (qu_s - qu) : qu_s - qu_e) < 18) add_list_memory(qu_s, qu_e);
				add_to_mask(ei, ej, ek, qu_e);
			}
		}
	private:
		/* A constant set to boxx*boxx+boxy*boxy+boxz*boxz, which is
		 * frequently used in the computation. */
		const double bxsq;
		/* This sets the current value being used to mark tested blocks
		 * in the mask. */
		unsigned int mv;
		/* The current size of the search list. */
		int qu_size;

		// TODO: can get via 'con' -- no reason for wl to live on this class?
		/* A pointer to the array of worklists. */
		const unsigned int* wl;
		/* An pointer to the array holding the minimum distances
		 * associated with the worklists. */
		double* mrad;
		/* This array is used during the cell computation to determine
		 * which blocks have been considered. */
		unsigned int* mask;
		/* An array is used to store the queue of blocks to test
		 * during the Voronoi cell computation. */
		int* qu;
		/* A pointer to the end of the queue array, used to determine
		 * when the queue is full. */
		int* qu_l;
		template<class v_cell>
		bool corner_test(v_cell& c, double xl, double yl, double zl, double xh, double yh, double zh, typename c_class::radius_info& radi) const
		{
			con.r_prime(xl * xl + yl * yl + zl * zl, radi);
			if (c.plane_intersects_guess(xh, yl, zl, con.r_cutoff(xl * xh + yl * yl + zl * zl, radi))) return false;
			if (c.plane_intersects(xh, yh, zl, con.r_cutoff(xl * xh + yl * yh + zl * zl, radi))) return false;
			if (c.plane_intersects(xl, yh, zl, con.r_cutoff(xl * xl + yl * yh + zl * zl, radi))) return false;
			if (c.plane_intersects(xl, yh, zh, con.r_cutoff(xl * xl + yl * yh + zl * zh, radi))) return false;
			if (c.plane_intersects(xl, yl, zh, con.r_cutoff(xl * xl + yl * yl + zl * zh, radi))) return false;
			if (c.plane_intersects(xh, yl, zh, con.r_cutoff(xl * xh + yl * yl + zl * zh, radi))) return false;
			return true;
		}

		template<class v_cell>
		inline bool edge_x_test(v_cell& c, double x0, double yl, double zl, double x1, double yh, double zh, typename c_class::radius_info& radi) const
		{
			con.r_prime(yl * yl + zl * zl, radi);
			if (c.plane_intersects_guess(x0, yl, zh, con.r_cutoff(yl * yl + zl * zh, radi))) return false;
			if (c.plane_intersects(x1, yl, zh, con.r_cutoff(yl * yl + zl * zh, radi))) return false;
			if (c.plane_intersects(x1, yl, zl, con.r_cutoff(yl * yl + zl * zl, radi))) return false;
			if (c.plane_intersects(x0, yl, zl, con.r_cutoff(yl * yl + zl * zl, radi))) return false;
			if (c.plane_intersects(x0, yh, zl, con.r_cutoff(yl * yh + zl * zl, radi))) return false;
			if (c.plane_intersects(x1, yh, zl, con.r_cutoff(yl * yh + zl * zl, radi))) return false;
			return true;
		}

		template<class v_cell>
		inline bool edge_y_test(v_cell& c, double xl, double y0, double zl, double xh, double y1, double zh, typename c_class::radius_info& radi) const
		{
			con.r_prime(xl * xl + zl * zl, radi);
			if (c.plane_intersects_guess(xl, y0, zh, con.r_cutoff(xl * xl + zl * zh, radi))) return false;
			if (c.plane_intersects(xl, y1, zh, con.r_cutoff(xl * xl + zl * zh, radi))) return false;
			if (c.plane_intersects(xl, y1, zl, con.r_cutoff(xl * xl + zl * zl, radi))) return false;
			if (c.plane_intersects(xl, y0, zl, con.r_cutoff(xl * xl + zl * zl, radi))) return false;
			if (c.plane_intersects(xh, y0, zl, con.r_cutoff(xl * xh + zl * zl, radi))) return false;
			if (c.plane_intersects(xh, y1, zl, con.r_cutoff(xl * xh + zl * zl, radi))) return false;
			return true;
		}

		template<class v_cell>
		inline bool edge_z_test(v_cell& c, double xl, double yl, double z0, double xh, double yh, double z1, typename c_class::radius_info& radi) const
		{
			con.r_prime(xl * xl + yl * yl, radi);
			if (c.plane_intersects_guess(xl, yh, z0, con.r_cutoff(xl * xl + yl * yh, radi))) return false;
			if (c.plane_intersects(xl, yh, z1, con.r_cutoff(xl * xl + yl * yh, radi))) return false;
			if (c.plane_intersects(xl, yl, z1, con.r_cutoff(xl * xl + yl * yl, radi))) return false;
			if (c.plane_intersects(xl, yl, z0, con.r_cutoff(xl * xl + yl * yl, radi))) return false;
			if (c.plane_intersects(xh, yl, z0, con.r_cutoff(xl * xh + yl * yl, radi))) return false;
			if (c.plane_intersects(xh, yl, z1, con.r_cutoff(xl * xh + yl * yl, radi))) return false;
			return true;
		}

		template<class v_cell>
		inline bool face_x_test(v_cell& c, double xl, double y0, double z0, double y1, double z1, typename c_class::radius_info& radi) const
		{
			con.r_prime(xl * xl, radi);
			if (c.plane_intersects_guess(xl, y0, z0, con.r_cutoff(xl * xl, radi))) return false;
			if (c.plane_intersects(xl, y0, z1, con.r_cutoff(xl * xl, radi))) return false;
			if (c.plane_intersects(xl, y1, z1, con.r_cutoff(xl * xl, radi))) return false;
			if (c.plane_intersects(xl, y1, z0, con.r_cutoff(xl * xl, radi))) return false;
			return true;
		}

		template<class v_cell>
		inline bool face_y_test(v_cell& c, double x0, double yl, double z0, double x1, double z1, typename c_class::radius_info& radi) const
		{
			con.r_prime(yl * yl, radi);
			if (c.plane_intersects_guess(x0, yl, z0, con.r_cutoff(yl * yl, radi))) return false;
			if (c.plane_intersects(x0, yl, z1, con.r_cutoff(yl * yl, radi))) return false;
			if (c.plane_intersects(x1, yl, z1, con.r_cutoff(yl * yl, radi))) return false;
			if (c.plane_intersects(x1, yl, z0, con.r_cutoff(yl * yl, radi))) return false;
			return true;
		}

		template<class v_cell>
		inline bool face_z_test(v_cell& c, double x0, double y0, double zl, double x1, double y1, typename c_class::radius_info& radi) const
		{
			con.r_prime(zl * zl, radi);
			if (c.plane_intersects_guess(x0, y0, zl, con.r_cutoff(zl * zl, radi))) return false;
			if (c.plane_intersects(x0, y1, zl, con.r_cutoff(zl * zl, radi))) return false;
			if (c.plane_intersects(x1, y1, zl, con.r_cutoff(zl * zl, radi))) return false;
			if (c.plane_intersects(x1, y0, zl, con.r_cutoff(zl * zl, radi))) return false;
			return true;
		}

		bool compute_min_max_radius(int di, int dj, int dk, double fx, double fy, double fz, double gxs, double gys, double gzs, double& crs, double mrs, const typename c_class::radius_info& radi) const
		{
			double xlo, ylo, zlo;
			if (di > 0) {
				xlo = di * boxx - fx;
				crs = xlo * xlo;
				if (dj > 0) {
					ylo = dj * boxy - fy;
					crs += ylo * ylo;
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (boxx * xlo + boxy * ylo + boxz * zlo);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (boxx * xlo + boxy * ylo - boxz * zlo);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxx * (2 * xlo + boxx) + boxy * (2 * ylo + boxy) + gzs;
					}
				}
				else if (dj < 0) {
					ylo = (dj + 1) * boxy - fy;
					crs += ylo * ylo;
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (boxx * xlo - boxy * ylo + boxz * zlo);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (boxx * xlo - boxy * ylo - boxz * zlo);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxx * (2 * xlo + boxx) + boxy * (-2 * ylo + boxy) + gzs;
					}
				}
				else {
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (2 * zlo + boxz);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (-2 * zlo + boxz);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += gzs;
					}
					crs += gys + boxx * (2 * xlo + boxx);
				}
			}
			else if (di < 0) {
				xlo = (di + 1) * boxx - fx;
				crs = xlo * xlo;
				if (dj > 0) {
					ylo = dj * boxy - fy;
					crs += ylo * ylo;
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (-boxx * xlo + boxy * ylo + boxz * zlo);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (-boxx * xlo + boxy * ylo - boxz * zlo);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxx * (-2 * xlo + boxx) + boxy * (2 * ylo + boxy) + gzs;
					}
				}
				else if (dj < 0) {
					ylo = (dj + 1) * boxy - fy;
					crs += ylo * ylo;
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (-boxx * xlo - boxy * ylo + boxz * zlo);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += bxsq + 2 * (-boxx * xlo - boxy * ylo - boxz * zlo);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxx * (-2 * xlo + boxx) + boxy * (-2 * ylo + boxy) + gzs;
					}
				}
				else {
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (2 * zlo + boxz);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (-2 * zlo + boxz);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += gzs;
					}
					crs += gys + boxx * (-2 * xlo + boxx);
				}
			}
			else {
				if (dj > 0) {
					ylo = dj * boxy - fy;
					crs = ylo * ylo;
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (2 * zlo + boxz);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (-2 * zlo + boxz);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += gzs;
					}
					crs += boxy * (2 * ylo + boxy);
				}
				else if (dj < 0) {
					ylo = (dj + 1) * boxy - fy;
					crs = ylo * ylo;
					if (dk > 0) {
						zlo = dk * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (2 * zlo + boxz);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz;
						crs += zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (-2 * zlo + boxz);
					}
					else {
						if (con.r_ctest(crs, mrs, radi)) return true;
						crs += gzs;
					}
					crs += boxy * (-2 * ylo + boxy);
				}
				else {
					if (dk > 0) {
						zlo = dk * boxz - fz; crs = zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (2 * zlo + boxz);
					}
					else if (dk < 0) {
						zlo = (dk + 1) * boxz - fz; crs = zlo * zlo; if (con.r_ctest(crs, mrs, radi)) return true;
						crs += boxz * (-2 * zlo + boxz);
					}
					else {
						crs = 0;
						assert(false);
					}
					crs += gys;
				}
				crs += gxs;
			}
			return false;
		}

		bool compute_min_radius(int di, int dj, int dk, double fx, double fy, double fz, double mrs) const
		{
			double t, crs;

			if (di > 0) { t = di * boxx - fx; crs = t * t; }
			else if (di < 0) { t = (di + 1) * boxx - fx; crs = t * t; }
			else crs = 0;

			if (dj > 0) { t = dj * boxy - fy; crs += t * t; }
			else if (dj < 0) { t = (dj + 1) * boxy - fy; crs += t * t; }

			if (dk > 0) { t = dk * boxz - fz; crs += t * t; }
			else if (dk < 0) { t = (dk + 1) * boxz - fz; crs += t * t; }

			return crs > con.r_max_add(mrs);
		}

		inline void add_to_mask(int ei, int ej, int ek, int*& qu_e)
		{
			unsigned int* mijk = mask + ei + hx * (ej + hy * ek);
			if (ek > 0) if (*(mijk - hxy) != mv) { if (qu_e == qu_l) qu_e = qu; *(mijk - hxy) = mv; *(qu_e++) = ei; *(qu_e++) = ej; *(qu_e++) = ek - 1; }
			if (ej > 0) if (*(mijk - hx) != mv) { if (qu_e == qu_l) qu_e = qu; *(mijk - hx) = mv; *(qu_e++) = ei; *(qu_e++) = ej - 1; *(qu_e++) = ek; }
			if (ei > 0) if (*(mijk - 1) != mv) { if (qu_e == qu_l) qu_e = qu; *(mijk - 1) = mv; *(qu_e++) = ei - 1; *(qu_e++) = ej; *(qu_e++) = ek; }
			if (ei < hx - 1) if (*(mijk + 1) != mv) { if (qu_e == qu_l) qu_e = qu; *(mijk + 1) = mv; *(qu_e++) = ei + 1; *(qu_e++) = ej; *(qu_e++) = ek; }
			if (ej < hy - 1) if (*(mijk + hx) != mv) { if (qu_e == qu_l) qu_e = qu; *(mijk + hx) = mv; *(qu_e++) = ei; *(qu_e++) = ej + 1; *(qu_e++) = ek; }
			if (ek < hz - 1) if (*(mijk + hxy) != mv) { if (qu_e == qu_l) qu_e = qu; *(mijk + hxy) = mv; *(qu_e++) = ei; *(qu_e++) = ej; *(qu_e++) = ek + 1; }
		}

		inline void scan_bits_mask_add(unsigned int q, unsigned int* mijk, int ei, int ej, int ek, int*& qu_e)
		{
			const unsigned int b1 = 1 << 21, b2 = 1 << 22, b3 = 1 << 24, b4 = 1 << 25, b5 = 1 << 27, b6 = 1 << 28;
			if ((q & b2) == b2) {
				if (ei > 0) { *(mijk - 1) = mv; *(qu_e++) = ei - 1; *(qu_e++) = ej; *(qu_e++) = ek; }
				if ((q & b1) == 0 && ei < hx - 1) { *(mijk + 1) = mv; *(qu_e++) = ei + 1; *(qu_e++) = ej; *(qu_e++) = ek; }
			}
			else if ((q & b1) == b1 && ei < hx - 1) { *(mijk + 1) = mv; *(qu_e++) = ei + 1; *(qu_e++) = ej; *(qu_e++) = ek; }
			if ((q & b4) == b4) {
				if (ej > 0) { *(mijk - hx) = mv; *(qu_e++) = ei; *(qu_e++) = ej - 1; *(qu_e++) = ek; }
				if ((q & b3) == 0 && ej < hy - 1) { *(mijk + hx) = mv; *(qu_e++) = ei; *(qu_e++) = ej + 1; *(qu_e++) = ek; }
			}
			else if ((q & b3) == b3 && ej < hy - 1) { *(mijk + hx) = mv; *(qu_e++) = ei; *(qu_e++) = ej + 1; *(qu_e++) = ek; }
			if ((q & b6) == b6) {
				if (ek > 0) { *(mijk - hxy) = mv; *(qu_e++) = ei; *(qu_e++) = ej; *(qu_e++) = ek - 1; }
				if ((q & b5) == 0 && ek < hz - 1) { *(mijk + hxy) = mv; *(qu_e++) = ei; *(qu_e++) = ej; *(qu_e++) = ek + 1; }
			}
			else if ((q & b5) == b5 && ek < hz - 1) { *(mijk + hxy) = mv; *(qu_e++) = ei; *(qu_e++) = ej; *(qu_e++) = ek + 1; }
		}

		inline void scan_all(int ijk, double x, double y, double z, int di, int dj, int dk, particle_record& w, double& mrs)
		{
			double x1, y1, z1, rs; bool in_block = false;
			for (int l = 0; l < co[ijk]; l++) {
				x1 = p[ijk][ps * l] - x;
				y1 = p[ijk][ps * l + 1] - y;
				z1 = p[ijk][ps * l + 2] - z;
				rs = con.r_current_sub(x1 * x1 + y1 * y1 + z1 * z1, ijk, l);
				if (rs < mrs) { mrs = rs; w.l = l; in_block = true; }
			}
            if (in_block) { w.ijk = ijk; w.di = di; w.dj = dj; w.dk = dk; }
		}

		void add_list_memory(int*& qu_s, int*& qu_e)
		{
			qu_size <<= 1;
			int* qu_n = new int[qu_size], * qu_c = qu_n;
			if (qu_s <= qu_e) {
				while (qu_s < qu_e) *(qu_c++) = *(qu_s++);
			}
			else {
				while (qu_s < qu_l) *(qu_c++) = *(qu_s++); qu_s = qu;
				while (qu_s < qu_e) *(qu_c++) = *(qu_s++);
			}
			delete[] qu;
			qu_s = qu = qu_n;
			qu_l = qu + qu_size;
			qu_e = qu_c;
		}

		/* Resets the mask in cases where the mask counter wraps
		 * around. */
		inline void reset_mask() {
			for (unsigned int* mp(mask); mp < mask + hxyz; mp++) *mp = 0;
		}
	};

	/* A type associated with a c_loop_subset class, determining what type of
 * geometrical region to loop over. */
	enum c_loop_subset_mode {
		sphere,
		box,
		no_check
	};

	/* \brief A class for storing ordering information when particles are added to
	 * a container.
	 *
	 * When particles are added to a container class, they are sorted into an
	 * internal computational grid of blocks. The particle_order class provides a
	 * mechanism for remembering which block particles were sorted into. The import
	 * and put routines in the container class have variants that also take a
	 * particle_order class. Each time they are called, they will store the block
	 * that the particle was sorted into, plus the position of the particle within
	 * the block. The particle_order class can used by the c_loop_order class to
	 * specifically loop over the particles that have their information stored
	 * within it. */
	class particle_order {
	public:
		/* A pointer to the array holding the ordering. */
		int* o;
		/* A pointer to the next position in the ordering array in
		 * which to store an entry. */
		int* op;
		/* The current memory allocation for the class, set to the
		 * number of entries which can be stored. */
		int size;
		/* The particle_order constructor allocates memory to store the
		 * ordering information.
		 * \param[in] init_size the initial amount of memory to
		 *                      allocate. */
		particle_order(int init_size = init_ordering_size)
			: o(new int[size_t(init_size) << 1]), op(o), size(init_size) {}
		/* The particle_order destructor frees the dynamically allocated
		 * memory used to store the ordering information. */
		~particle_order() {
			delete[] o;
		}
		/* Adds a record to the order, corresponding to the memory
		 * address of where a particle was placed into the container.
		 * \param[in] ijk the block into which the particle was placed.
		 * \param[in] q the position within the block where the
		 * 		particle was placed. */
		inline void add(int ijk, int q) {
			if (op == o + size) add_ordering_memory();
			*(op++) = ijk; *(op++) = q;
		}
	private:
		void add_ordering_memory()
		{
			int* no = new int[size_t(size) << 2], * nop = no, * opp = o;
			while (opp < op) *(nop++) = *(opp++);
			delete[] o;
			size <<= 1; o = no; op = nop;
		}
	};

	class c_loop_base {
	public:
		/* The number of blocks in the x direction. */
		const int nx;
		/* The number of blocks in the y direction. */
		const int ny;
		/* The number of blocks in the z direction. */
		const int nz;
		/* A constant, set to the value of nx multiplied by ny, which
		 * is used in the routines that step through blocks in
		 * sequence. */
		const int nxy;
		/* A constant, set to the value of nx*ny*nz, which is used in
		 * the routines that step through blocks in sequence. */
		const int nxyz;
		/* The number of floating point numbers per particle in the
		 * associated container data structure. */
		const int ps;
		/* A pointer to the particle position information in the
		 * associated container data structure. */
		double** p;
		/* A pointer to the particle ID information in the associated
		 * container data structure. */
		int** id;
		/* A pointer to the particle counts in the associated
		 * container data structure. */
		int* co;
		/* The current x-index of the block under consideration by the
		 * loop. */
		int i;
		/* The current y-index of the block under consideration by the
		 * loop. */
		int j;
		/* The current z-index of the block under consideration by the
		 * loop. */
		int k;
		/* The current index of the block under consideration by the
		 * loop. */
		int ijk;
		/* The index of the particle under consideration within the current
		 * block. */
		int q;
		/* The constructor copies several necessary constants from the
		 * base container class.
		 * \param[in] con the container class to use. */
		template<class c_class>
		c_loop_base(c_class& con) : nx(con.nx), ny(con.ny), nz(con.nz),
			nxy(con.nxy), nxyz(con.nxyz), ps(con.ps),
			p(con.p), id(con.id), co(con.co) {}
		/* Returns the position vector of the particle currently being
		 * considered by the loop.
		 * \param[out] (x,y,z) the position vector of the particle. */
		inline void pos(double& x, double& y, double& z) {
			double* pp = p[ijk] + ps * q;
			x = *(pp++); y = *(pp++); z = *pp;
		}
		/* Returns the ID, position vector, and radius of the particle
		 * currently being considered by the loop.
		 * \param[out] pid the particle ID.
		 * \param[out] (x,y,z) the position vector of the particle.
		 * \param[out] r the radius of the particle. If no radius
		 * 		 information is available the default radius
		 * 		 value is returned. */
		inline void pos(int& pid, double& x, double& y, double& z, double& r) {
			pid = id[ijk][q];
			double* pp = p[ijk] + ps * q;
			x = *(pp++); y = *(pp++); z = *pp;
			r = ps == 3 ? default_radius : *(++pp);
		}
		/* Returns the x position of the particle currently being
		 * considered by the loop. */
		inline double x() { return p[ijk][ps * q]; }
		/* Returns the y position of the particle currently being
		 * considered by the loop. */
		inline double y() { return p[ijk][ps * q + 1]; }
		/* Returns the z position of the particle currently being
		 * considered by the loop. */
		inline double z() { return p[ijk][ps * q + 2]; }
		/* Returns the ID of the particle currently being considered
		 * by the loop. */
		inline int pid() { return id[ijk][q]; }
	};

	/* \brief Class for looping over all of the particles in a container.
	 *
	 * This is one of the simplest loop classes, that scans the computational
	 * blocks in order, and scans all the particles within each block in order. */
	class c_loop_all : public c_loop_base {
	public:
		/* The constructor copies several necessary constants from the
		 * base container class.
		 * \param[in] con the container class to use. */
		template<class c_class>
		c_loop_all(c_class& con) : c_loop_base(con) {}
		/* Sets the class to consider the first particle.
		 * \return True if there is any particle to consider, false
		 * otherwise. */
		inline bool start() {
			i = j = k = ijk = q = 0;
			while (co[ijk] == 0) if (!next_block()) return false;
			return true;
		}
		/* Finds the next particle to test.
		 * \return True if there is another particle, false if no more
		 * particles are available. */
		inline bool inc() {
			q++;
			if (q >= co[ijk]) {
				q = 0;
				do {
					if (!next_block()) return false;
				} while (co[ijk] == 0);
			}
			return true;
		}
	private:
		/* Updates the internal variables to find the next
		 * computational block with any particles.
		 * \return True if another block is found, false if there are
		 * no more blocks. */
		inline bool next_block() {
			ijk++;
			i++;
			if (i == nx) {
				i = 0; j++;
				if (j == ny) {
					j = 0; k++;
					if (ijk == nxyz) return false;
				}
			}
			return true;
		}
	};

	/* \brief Class for looping over the particles in a range of blocks within a container.
 *
 * This scans the computational blocks in order, and scans all the particles within each block in order. */
	class c_loop_block_range : public c_loop_base {
	public:
		/* The constructor copies several necessary constants from the
		 * base container class.  Defaults to full range.
		 * \param[in] con the container class to use. */
		template<class c_class>
		c_loop_block_range(c_class& con) : c_loop_base(con) {}
		/* Sets the class to consider the first particle.
		 * \return True if there is any particle to consider, false
		 * otherwise. */
		inline bool start() {
			ijk = ijk_start;
			assert(ijk >= 0 && ijk < nxyz);
			k = ijk / nxy;
			int ijkt = ijk - nxy * k;
			j = ijkt / nx;
			i = ijkt - j * nx;
			q = 0;
			while (co[ijk] == 0) if (!next_block()) return false;
			return true;
		}
		void setup_range(int ijk_start_in, int ijk_stop_in)
		{
			ijk_start = ijk_start_in;
			ijk_stop = ijk_stop_in;

			assert(ijk_start >= 0 && ijk_start < nxyz&& ijk_stop >= 0 && ijk_stop <= nxyz);
			assert(ijk_start < ijk_stop);
		}
		/* Finds the next particle to test.
		 * \return True if there is another particle, false if no more
		 * particles are available. */
		inline bool inc() {
			q++;
			if (q >= co[ijk]) {
				q = 0;
				do {
					if (!next_block()) return false;
				} while (co[ijk] == 0);
			}
			return true;
		}
	private:
		int ijk_start = 0, ijk_stop = 0;

		/* Updates the internal variables to find the next
		 * computational block with any particles.
		 * \return True if another block is found, false if there are
		 * no more blocks. */
		inline bool next_block() {
			ijk++;
			if (ijk >= ijk_stop) return false;
			i++;
			if (i == nx) {
				i = 0; j++;
				if (j == ny) {
					j = 0; k++;
				}
			}
			return true;
		}
	};

	/* \brief Pure virtual class from which wall objects are derived.
	 *
	 * This is a pure virtual class for a generic wall object. A wall object
	 * can be specified by deriving a new class from this and specifying the
	 * functions.*/
	class wall {
	public:
		virtual ~wall() {}
		/* A pure virtual function for testing whether a point is
		 * inside the wall object. */
		virtual bool point_inside(double x, double y, double z) const = 0;
		/* A pure virtual function for cutting a cell without
		 * neighbor-tracking with a wall. */
		virtual bool cut_cell(voronoicell& c, double x, double y, double z) const = 0;
		/* A pure virtual function for cutting a cell with
		 * neighbor-tracking enabled with a wall. */
		virtual bool cut_cell(voronoicell_neighbor& c, double x, double y, double z) const = 0;
	};

	/* \brief A class for storing a list of pointers to walls.
	 *
	 * This class stores a list of pointers to wall classes. It contains several
	 * simple routines that make use of the wall classes (such as telling whether a
	 * given position is inside all of the walls or not). It can be used by itself,
	 * but also forms part of container_base, for associating walls with this
	 * class. */
	class wall_list {
	public:
		/* An array holding pointers to wall objects. */
		wall** walls;
		/* A pointer to the next free position to add a wall pointer.
		 */
		wall** wep;
		wall_list()
		 : walls(new wall*[init_wall_size]), wep(walls), wel(walls+init_wall_size),
			current_wall_size(init_wall_size) {}
		wall_list(const wall_list& Other) = delete;
		wall_list& operator=(const wall_list& Other) = delete;
		wall_list(wall_list&& Other) noexcept : walls(Other.walls), wep(Other.wep), wel(Other.wel), current_wall_size(Other.current_wall_size)
		{
			Other.walls = nullptr;
		}
		wall_list& operator=(wall_list&& Other)
		{
			if (this != &Other)
			{
				walls = Other.walls;
				wep = Other.wep;
				wel = Other.wel;
				current_wall_size = Other.current_wall_size;
				Other.walls = nullptr;
			}
			return *this;
		}
		~wall_list()
		{
			delete[] walls;
		}
		/* Adds a wall to the list.
		 * \param[in] w the wall to add. */
		inline void add_wall(wall* w) {
			if (wep == wel) increase_wall_memory();
			*(wep++) = w;
		}
		/* Adds a wall to the list.
		 * \param[in] w a reference to the wall to add. */
		inline void add_wall(wall& w) { add_wall(&w); }

		void add_wall(wall_list& wl) { for (wall** wp = wl.walls; wp < wl.wep; wp++) add_wall(*wp); }

		/* Determines whether a given position is inside all of the
		 * walls on the list.
		 * \param[in] (x,y,z) the position to test.
		 * \return True if it is inside, false if it is outside. */
		inline bool point_inside_walls(double x, double y, double z) const {
			for (wall** wp = walls; wp < wep; wp++) if (!((*wp)->point_inside(x, y, z))) return false;
			return true;
		}

		/* Cuts a Voronoi cell by all of the walls currently on
		 * the list.
		 * \param[in] c a reference to the Voronoi cell class.
		 * \param[in] (x,y,z) the position of the cell.
		 * \return True if the cell still exists, false if the cell is
		 * deleted. */
		template<class c_class>
		bool apply_walls(c_class& c, double x, double y, double z) const {
			for (wall** wp = walls; wp < wep; wp++) if (!((*wp)->cut_cell(c, x, y, z))) return false;
			return true;
		}

		void deallocate() { for (wall** wp = walls; wp < wep; wp++) delete* wp; }

	protected:
		void increase_wall_memory()
		{
			current_wall_size <<= 1;
			if (current_wall_size > max_wall_size)
			{
				assert(false);
			}
			wall** nwalls = new wall * [current_wall_size], ** nwp = nwalls, ** wp = walls;
			while (wp < wep) *(nwp++) = *(wp++);
			delete[] walls;
			walls = nwalls; wel = walls + current_wall_size; wep = nwp;
		}

		/* A pointer to the limit of the walls array, used to
		 * determine when array is full. */
		wall** wel;
		/* The current amount of memory allocated for walls. */
		int current_wall_size;
	};

	/* \brief Class for representing a particle system in a three-dimensional
	 * rectangular box.
	 *
	 * This class represents a system of particles in a three-dimensional
	 * rectangular box. Any combination of non-periodic and periodic coordinates
	 * can be used in the three coordinate directions. The class is not intended
	 * for direct use, but instead forms the base of the container and
	 * container_poly classes that add specialized routines for computing the
	 * regular and radical Voronoi tessellations respectively. It contains routines
	 * that are commonly between these two classes, such as those for drawing the
	 * domain, and placing particles within the internal data structure.
	 *
	 * The class is derived from the wall_list class, which encapsulates routines
	 * for associating walls with the container, and the voro_base class, which
	 * encapsulates routines about the underlying computational grid. */
	class container_base : public voro_base, public wall_list {
	public:
		/* The minimum x coordinate of the container. */
		const double ax;
		/* The maximum x coordinate of the container. */
		const double bx;
		/* The minimum y coordinate of the container. */
		const double ay;
		/* The maximum y coordinate of the container. */
		const double by;
		/* The minimum z coordinate of the container. */
		const double az;
		/* The maximum z coordinate of the container. */
		const double bz;
		/* A boolean value that determines if the x coordinate in
		 * periodic or not. */
		const bool xperiodic;
		/* A boolean value that determines if the y coordinate in
		 * periodic or not. */
		const bool yperiodic;
		/* A boolean value that determines if the z coordinate in
		 * periodic or not. */
		const bool zperiodic;
		/* This array holds the numerical IDs of each particle in each
		 * computational box. */
		int** id;
		/* A two dimensional array holding particle positions. For the
		 * derived container_poly class, this also holds particle
		 * radii. */
		double** p;
		/* This array holds the number of particles within each
		 * computational box of the container. */
		int* co;
		/* This array holds the maximum amount of particle memory for
		 * each computational box of the container. If the number of
		 * particles in a particular box ever approaches this limit,
		 * more is allocated using the add_particle_memory() function.
		 */
		int* mem;
		/* The amount of memory in the array structure for each
		 * particle. This is set to 3 when the basic class is
		 * initialized, so that the array holds (x,y,z) positions. If
		 * the container class is initialized as part of the derived
		 * class container_poly, then this is set to 4, to also hold
		 * the particle radii. */
		const int ps;
		container_base(double ax_, double bx_, double ay_, double by_, double az_, double bz_,
			int nx_, int ny_, int nz_, bool xperiodic_, bool yperiodic_, bool zperiodic_,
			int init_mem, int ps_)
		: voro_base(nx_, ny_, nz_, (bx_ - ax_) / nx_, (by_ - ay_) / ny_, (bz_ - az_) / nz_),
			ax(ax_), bx(bx_), ay(ay_), by(by_), az(az_), bz(bz_),
			xperiodic(xperiodic_), yperiodic(yperiodic_), zperiodic(zperiodic_),
			id(new int* [nxyz]), p(new double* [nxyz]), co(new int[nxyz]), mem(new int[nxyz]), ps(ps_) {
			int l;
			for (l = 0; l < nxyz; l++) co[l] = 0;
			for (l = 0; l < nxyz; l++) mem[l] = init_mem;
			for (l = 0; l < nxyz; l++) id[l] = new int[init_mem];
			for (l = 0; l < nxyz; l++) p[l] = new double[ps * init_mem];
		}

		container_base(container_base& Other) = delete;
		container_base& operator=(const container_base& Other) = delete;
		container_base(container_base&& Other) noexcept : voro_base(std::move(Other)), wall_list(std::move(Other)),
			ax(Other.ax), bx(Other.bx), ay(Other.ay), by(Other.by), az(Other.az), bz(Other.bz),
			xperiodic(Other.xperiodic), yperiodic(Other.yperiodic), zperiodic(Other.zperiodic),
			id(Other.id), p(Other.p), co(Other.co), mem(Other.mem), ps(Other.ps)
		{
			Other.id = nullptr;
			Other.p = nullptr;
			Other.co = nullptr;
			Other.mem = nullptr;
		}
		// Note: Move assignment not implemented due to const members on this class (and it has not been needed yet)
		container_base& operator=(container_base&& Other) = delete;

		~container_base()
		{
			int l;
			if (p)
			{
				for (l = 0; l < nxyz; l++) delete[] p[l];
				delete[] p;
			}
			if (id)
			{
				for (l = 0; l < nxyz; l++) delete[] id[l];
				delete[] id;
			}
			delete[] co;
			delete[] mem;
		}

		bool point_inside(double x, double y, double z) const
		{
			if (x<ax || x>bx || y<ay || y>by || z<az || z>bz) return false;
			return point_inside_walls(x, y, z);
		}

		void region_count() const
		{
			int i, j, k, * cop = co;
			for (k = 0; k < nz; k++) for (j = 0; j < ny; j++) for (i = 0; i < nx; i++)
				printf("Region (%d,%d,%d): %d particles\n", i, j, k, *(cop++));
		}

		/* Initializes the Voronoi cell prior to a compute_cell
		 * operation for a specific particle being carried out by a
		 * voro_compute class. The cell is initialized to fill the
		 * entire container. For non-periodic coordinates, this is set
		 * by the position of the walls. For periodic coordinates, the
		 * space is equally divided in either direction from the
		 * particle's initial position. Plane cuts made by any walls
		 * that have been added are then applied to the cell.
		 * \param[in,out] c a reference to a voronoicell object.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within its block.
		 * \param[in] (ci,cj,ck) the coordinates of the block in the
		 * 			 container coordinate system.
		 * \param[out] (i,j,k) the coordinates of the test block
		 * 		       relative to the voro_compute
		 * 		       coordinate system.
		 * \param[out] (x,y,z) the position of the particle.
		 * \param[out] disp a block displacement used internally by the
		 *		    compute_cell routine.
		 * \return False if the plane cuts applied by walls completely
		 * removed the cell, true otherwise. */
		template<class v_cell>
		inline bool initialize_voronoicell(v_cell& c, int ijk, int q, int ci, int cj, int ck,
			int& i, int& j, int& k, double& x, double& y, double& z, int& disp) const {
			double x1, x2, y1, y2, z1, z2, * pp = p[ijk] + ps * q;
			x = *(pp++); y = *(pp++); z = *pp;
			if (xperiodic) { x1 = -(x2 = 0.5 * (bx - ax)); i = nx; }
			else { x1 = ax - x; x2 = bx - x; i = ci; }
			if (yperiodic) { y1 = -(y2 = 0.5 * (by - ay)); j = ny; }
			else { y1 = ay - y; y2 = by - y; j = cj; }
			if (zperiodic) { z1 = -(z2 = 0.5 * (bz - az)); k = nz; }
			else { z1 = az - z; z2 = bz - z; k = ck; }
			c.init(x1, x2, y1, y2, z1, z2);
			if (!apply_walls(c, x, y, z)) return false;
			disp = ijk - i - nx * (j + ny * k);
			return true;
		}

		/* Initializes parameters for a find_voronoi_cell call within
		 * the voro_compute template.
		 * \param[in] (ci,cj,ck) the coordinates of the test block in
		 * 			 the container coordinate system.
		 * \param[in] ijk the index of the test block
		 * \param[out] (i,j,k) the coordinates of the test block
		 * 		       relative to the voro_compute
		 * 		       coordinate system.
		 * \param[out] disp a block displacement used internally by the
		 *		    find_voronoi_cell routine. */
		inline void initialize_search(int ci, int cj, int ck, int ijk, int& i, int& j, int& k, int& disp) const {
			i = xperiodic ? nx : ci;
			j = yperiodic ? ny : cj;
			k = zperiodic ? nz : ck;
			disp = ijk - i - nx * (j + ny * k);
		}

		/* Returns the position of a particle currently being computed
		 * relative to the computational block that it is within. It is
		 * used to select the optimal worklist entry to use.
		 * \param[in] (x,y,z) the position of the particle.
		 * \param[in] (ci,cj,ck) the block that the particle is within.
		 * \param[out] (fx,fy,fz) the position relative to the block.
		 */
		inline void frac_pos(double x, double y, double z, double ci, double cj, double ck,
			double& fx, double& fy, double& fz) const {
			fx = x - ax - boxx * ci;
			fy = y - ay - boxy * cj;
			fz = z - az - boxz * ck;
		}

		/* Calculates the index of block in the container structure
		 * corresponding to given coordinates.
		 * \param[in] (ci,cj,ck) the coordinates of the original block
		 * 			 in the current computation, relative
		 * 			 to the container coordinate system.
		 * \param[in] (ei,ej,ek) the displacement of the current block
		 * 			 from the original block.
		 * \param[in,out] (qx,qy,qz) the periodic displacement that
		 * 			     must be added to the particles
		 * 			     within the computed block.
		 * \param[in] disp a block displacement used internally by the
		 * 		    find_voronoi_cell and compute_cell routines.
		 * \return The block index. */
		inline int region_index(int ci, int cj, int ck, int ei, int ej, int ek, double& qx, double& qy, double& qz, int& disp) const {
			if (xperiodic) { if (ci + ei < nx) { ei += nx; qx = -(bx - ax); } else if (ci + ei >= (nx << 1)) { ei -= nx; qx = bx - ax; } else qx = 0; }
			if (yperiodic) { if (cj + ej < ny) { ej += ny; qy = -(by - ay); } else if (cj + ej >= (ny << 1)) { ej -= ny; qy = by - ay; } else qy = 0; }
			if (zperiodic) { if (ck + ek < nz) { ek += nz; qz = -(bz - az); } else if (ck + ek >= (nz << 1)) { ek -= nz; qz = bz - az; } else qz = 0; }
			return disp + ei + nx * (ej + ny * ek);
		}

		/* Sums up the total number of stored particles.
		 * \return The number of particles. */
		inline int total_particles() const {
			int tp = *co;
			for (int* cop = co + 1; cop < co + nxyz; cop++) tp += *cop;
			return tp;
		}

	protected:
		void add_particle_memory(int i)
		{
			int l, nmem = mem[i] << 1;

			// Carry out a check on the memory allocation size, and
			// print a status message if requested
			if (nmem > max_particle_memory)
			{
				assert(false);
			}
			// Allocate new memory and copy in the contents of the old arrays
			int* idp = new int[nmem];
			for (l = 0; l < co[i]; l++) idp[l] = id[i][l];
			double* pp = new double[ps * nmem];
			for (l = 0; l < ps * co[i]; l++) pp[l] = p[i][l];

			// Update pointers and delete old arrays
			mem[i] = nmem;
			delete[] id[i]; id[i] = idp;
			delete[] p[i]; p[i] = pp;
		}

		bool put_locate_block(int& ijk, double& x, double& y, double& z)
		{
			if (put_remap(ijk, x, y, z)) {
				if (co[ijk] == mem[ijk]) add_particle_memory(ijk);
				return true;
			}
			return false;
		}

		inline bool put_remap(int& ijk, double& x, double& y, double& z) const
		{
			int l;

			ijk = step_int((x - ax) * xsp);
			if (xperiodic) { l = step_mod(ijk, nx); x += boxx * (l - ijk); ijk = l; }
			else if (ijk < 0 || ijk >= nx) return false;

			int j = step_int((y - ay) * ysp);
			if (yperiodic) { l = step_mod(j, ny); y += boxy * (l - j); j = l; }
			else if (j < 0 || j >= ny) return false;

			int k = step_int((z - az) * zsp);
			if (zperiodic) { l = step_mod(k, nz); z += boxz * (l - k); k = l; }
			else if (k < 0 || k >= nz) return false;

			ijk += nx * j + nxy * k;
			return true;
		}

		inline bool remap(int& ai, int& aj, int& ak, int& ci, int& cj, int& ck, double& x, double& y, double& z, int& ijk) const
		{
			ci = step_int((x - ax) * xsp);
			if (ci < 0 || ci >= nx) {
				if (xperiodic) { ai = step_div(ci, nx); x -= ai * (bx - ax); ci -= ai * nx; }
				else return false;
			}
			else ai = 0;

			cj = step_int((y - ay) * ysp);
			if (cj < 0 || cj >= ny) {
				if (yperiodic) { aj = step_div(cj, ny); y -= aj * (by - ay); cj -= aj * ny; }
				else return false;
			}
			else aj = 0;

			ck = step_int((z - az) * zsp);
			if (ck < 0 || ck >= nz) {
				if (zperiodic) { ak = step_div(ck, nz); z -= ak * (bz - az); ck -= ak * nz; }
				else return false;
			}
			else ak = 0;

			ijk = ci + nx * cj + nxy * ck;
			return true;
		}
	};

	/* \brief Extension of the container_base class for computing regular Voronoi
	 * tessellations.
	 *
	 * This class is an extension of the container_base class that has routines
	 * specifically for computing the regular Voronoi tessellation with no
	 * dependence on particle radii. */
	class container : public container_base, public radius_mono {
	public:
		container(double ax_, double bx_, double ay_, double by_, double az_, double bz_,
			int nx_, int ny_, int nz_, bool xperiodic_, bool yperiodic_, bool zperiodic_, int init_mem)
			: container_base(ax_, bx_, ay_, by_, az_, bz_, nx_, ny_, nz_, xperiodic_, yperiodic_, zperiodic_, init_mem, 3) {}
		voro_compute<container> make_compute() const
		{
			return voro_compute<container>((container&)*this, xperiodic ? 2 * nx + 1 : nx, yperiodic ? 2 * ny + 1 : ny, zperiodic ? 2 * nz + 1 : nz);
		}

		void clear()
		{
			for (int* cop = co; cop < co + nxyz; cop++) *cop = 0;
		}

		void put(int n, double x, double y, double z)
		{
			int ijk;
			if (put_locate_block(ijk, x, y, z)) {
				id[ijk][co[ijk]] = n;
				double* pp = p[ijk] + 3 * co[ijk]++;
				*(pp++) = x; *(pp++) = y; *pp = z;
			}
		}

		void put(particle_order& vo, int n, double x, double y, double z)
		{
			int ijk;
			if (put_locate_block(ijk, x, y, z)) {
				id[ijk][co[ijk]] = n;
				vo.add(ijk, co[ijk]);
				double* pp = p[ijk] + 3 * co[ijk]++;
				*(pp++) = x; *(pp++) = y; *pp = z;
			}
		}

		void compute_all_cells(voro_compute<container>& vc)
		{
			voronoicell c;
			c_loop_all vl(*this);
			if (vl.start()) do compute_cell(c, vl, vc);
			while (vl.inc());
		}

		double sum_cell_volumes(voro_compute<container>& vc)
		{
			voronoicell c;
			double vol = 0;
			c_loop_all vl(*this);
			if (vl.start()) do if (compute_cell(c, vl, vc)) vol += c.volume(); while (vl.inc());
			return vol;
		}

		bool find_voronoi_cell(double x, double y, double z, double& rx, double& ry, double& rz, int& pid, voro_compute<container>& vc) const
		{
			int ai, aj, ak, ci, cj, ck, ijk;
			particle_record w;
			double mrs;

			// If the given vector lies outside the domain, but the container
			// is periodic, then remap it back into the domain
			if (!remap(ai, aj, ak, ci, cj, ck, x, y, z, ijk)) return false;
			vc.find_voronoi_cell(x, y, z, ci, cj, ck, ijk, w, mrs);

			if (w.ijk != -1) {

				// Assemble the position vector of the particle to be returned,
				// applying a periodic remapping if necessary
				if (xperiodic) { ci += w.di; if (ci < 0 || ci >= nx) ai += step_div(ci, nx); }
				if (yperiodic) { cj += w.dj; if (cj < 0 || cj >= ny) aj += step_div(cj, ny); }
				if (zperiodic) { ck += w.dk; if (ck < 0 || ck >= nz) ak += step_div(ck, nz); }
				rx = p[w.ijk][3 * w.l] + ai * (bx - ax);
				ry = p[w.ijk][3 * w.l + 1] + aj * (by - ay);
				rz = p[w.ijk][3 * w.l + 2] + ak * (bz - az);
				pid = id[w.ijk][w.l];
				return true;
			}

			// If no particle if found then just return false
			return false;
		}

		/* Computes the Voronoi cell for a particle currently being
		 * referenced by a loop class.
		 * \param[out] c a Voronoi cell class in which to store the
		 * 		 computed cell.
		 * \param[in] vl the loop class to use.
		 * \return True if the cell was computed. If the cell cannot be
		 * computed, if it is removed entirely by a wall or boundary
		 * condition, then the routine returns false. */
		template<class v_cell, class c_loop>
		inline bool compute_cell(v_cell& c, c_loop& vl, voro_compute<container>& vc) const {
			return vc.compute_cell(c, vl.ijk, vl.q, vl.i, vl.j, vl.k);
		}
		/* Computes the Voronoi cell for given particle.
		 * \param[out] c a Voronoi cell class in which to store the
		 * 		 computed cell.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return True if the cell was computed. If the cell cannot be
		 * computed, if it is removed entirely by a wall or boundary
		 * condition, then the routine returns false. */
		template<class v_cell>
		inline bool compute_cell(v_cell& c, int ijk, int q, voro_compute<container>& vc) const {
			int k = ijk / nxy, ijkt = ijk - nxy * k, j = ijkt / nx, i = ijkt - j * nx;
			return vc.compute_cell(c, ijk, q, i, j, k);
		}
		/* Computes the Voronoi cell for a ghost particle at a given
		 * location. NOT thread-safe; temporarily places particle into the container!
		 * \param[out] c a Voronoi cell class in which to store the
		 * 		 computed cell.
		 * \param[in] (x,y,z) the location of the ghost particle.
		 * \return True if the cell was computed. If the cell cannot be
		 * computed, if it is removed entirely by a wall or boundary
		 * condition, then the routine returns false. */
		template<class v_cell>
		inline bool compute_ghost_cell(v_cell& c, double x, double y, double z, voro_compute<container>& vc) {
			int ijk;
			if (put_locate_block(ijk, x, y, z)) {
				double* pp = p[ijk] + 3 * co[ijk]++;
				*(pp++) = x; *(pp++) = y; *pp = z;
				bool q = compute_cell(c, ijk, co[ijk] - 1, vc);
				co[ijk]--;
				return q;
			}
			return false;
		}
	private:
		friend class voro_compute<container>;
	};

	/* \brief Extension of the container_base class for computing radical Voronoi
	 * tessellations.
	 *
	 * This class is an extension of container_base class that has routines
	 * specifically for computing the radical Voronoi tessellation that depends on
	 * the particle radii. */
	class container_poly : public container_base, public radius_poly {
	public:
		container_poly(double ax_, double bx_, double ay_, double by_, double az_, double bz_,
			int nx_, int ny_, int nz_, bool xperiodic_, bool yperiodic_, bool zperiodic_, int init_mem)
			: container_base(ax_, bx_, ay_, by_, az_, bz_, nx_, ny_, nz_, xperiodic_, yperiodic_, zperiodic_, init_mem, 4) {
			ppr = p;
		}
		voro_compute<container_poly> make_compute() const
		{
			return voro_compute<container_poly>((container_poly&)*this, xperiodic ? 2 * nx + 1 : nx, yperiodic ? 2 * ny + 1 : ny, zperiodic ? 2 * nz + 1 : nz);
		}

		void clear()
		{
			for (int* cop = co; cop < co + nxyz; cop++) *cop = 0;
			max_radius = 0;
		}

		void put(int n, double x, double y, double z, double r)
		{
			int ijk;
			if (put_locate_block(ijk, x, y, z)) {
				id[ijk][co[ijk]] = n;
				double* pp = p[ijk] + 4 * co[ijk]++;
				*(pp++) = x; *(pp++) = y; *(pp++) = z; *pp = r;
				if (max_radius < r) max_radius = r;
			}
		}

		void put(particle_order& vo, int n, double x, double y, double z, double r)
		{
			int ijk;
			if (put_locate_block(ijk, x, y, z)) {
				id[ijk][co[ijk]] = n;
				vo.add(ijk, co[ijk]);
				double* pp = p[ijk] + 4 * co[ijk]++;
				*(pp++) = x; *(pp++) = y; *(pp++) = z; *pp = r;
				if (max_radius < r) max_radius = r;
			}
		}

		void compute_all_cells(voro_compute<container>& vc)
		{
			voronoicell c;
			c_loop_all vl(*this);
			if (vl.start()) do compute_cell(c, vl, vc); while (vl.inc());
		}

		double sum_cell_volumes(voro_compute<container>& vc)
		{
			voronoicell c;
			double vol = 0;
			c_loop_all vl(*this);
			if (vl.start()) do if (compute_cell(c, vl, vc)) vol += c.volume(); while (vl.inc());
			return vol;
		}
		/* Computes the Voronoi cell for a particle currently being
		 * referenced by a loop class.
		 * \param[out] c a Voronoi cell class in which to store the
		 * 		 computed cell.
		 * \param[in] vl the loop class to use.
		 * \return True if the cell was computed. If the cell cannot be
		 * computed, if it is removed entirely by a wall or boundary
		 * condition, then the routine returns false. */
		template<class v_cell, class c_loop>
		inline bool compute_cell(v_cell& c, c_loop& vl, voro_compute<container>& vc) const {
			return vc.compute_cell(c, vl.ijk, vl.q, vl.i, vl.j, vl.k);
		}
		/* Computes the Voronoi cell for given particle.
		 * \param[out] c a Voronoi cell class in which to store the
		 * 		 computed cell.
		 * \param[in] ijk the block that the particle is within.
		 * \param[in] q the index of the particle within the block.
		 * \return True if the cell was computed. If the cell cannot be
		 * computed, if it is removed entirely by a wall or boundary
		 * condition, then the routine returns false. */
		template<class v_cell>
		inline bool compute_cell(v_cell& c, int ijk, int q, voro_compute<container>& vc) const {
			int k = ijk / nxy, ijkt = ijk - nxy * k, j = ijkt / nx, i = ijkt - j * nx;
			return vc.compute_cell(c, ijk, q, i, j, k);
		}
		/* Computes the Voronoi cell for a ghost particle at a given
		 * location.
		 * \param[out] c a Voronoi cell class in which to store the
		 * 		 computed cell. NOT thread-safe; temporarily places particle into the container!
		 * \param[in] (x,y,z) the location of the ghost particle.
		 * \param[in] r the radius of the ghost particle.
		 * \return True if the cell was computed. If the cell cannot be
		 * computed, if it is removed entirely by a wall or boundary
		 * condition, then the routine returns false. */
		template<class v_cell>
		inline bool compute_ghost_cell(v_cell& c, double x, double y, double z, double r, voro_compute<container>& vc) {
			int ijk;
			if (put_locate_block(ijk, x, y, z)) {
				double* pp = p[ijk] + 4 * co[ijk]++, tm = max_radius;
				*(pp++) = x; *(pp++) = y; *(pp++) = z; *pp = r;
				if (r > max_radius) max_radius = r;
				bool q = compute_cell(c, ijk, co[ijk] - 1);
				co[ijk]--; max_radius = tm;
				return q;
			}
			return false;
		}

		bool find_voronoi_cell(double x, double y, double z, double& rx, double& ry, double& rz, int& pid, voro_compute<container>& vc) const
		{
			int ai, aj, ak, ci, cj, ck, ijk;
			particle_record w;
			double mrs;

			// If the given vector lies outside the domain, but the container
			// is periodic, then remap it back into the domain
			if (!remap(ai, aj, ak, ci, cj, ck, x, y, z, ijk)) return false;
			vc.find_voronoi_cell(x, y, z, ci, cj, ck, ijk, w, mrs);

			if (w.ijk != -1) {

				// Assemble the position vector of the particle to be returned,
				// applying a periodic remapping if necessary
				if (xperiodic) { ci += w.di; if (ci < 0 || ci >= nx) ai += step_div(ci, nx); }
				if (yperiodic) { cj += w.dj; if (cj < 0 || cj >= ny) aj += step_div(cj, ny); }
				if (zperiodic) { ck += w.dk; if (ck < 0 || ck >= nz) ak += step_div(ck, nz); }
				rx = p[w.ijk][4 * w.l] + ai * (bx - ax);
				ry = p[w.ijk][4 * w.l + 1] + aj * (by - ay);
				rz = p[w.ijk][4 * w.l + 2] + ak * (bz - az);
				pid = id[w.ijk][w.l];
				return true;
			}

			// If no particle if found then just return false
			return false;
		}

	private:
		friend class voro_compute<container_poly>;
	};

	void guess_optimal(int siteCount, float sizeX, float sizeY, float sizeZ, int& nx, int& ny, int& nz) {
		float ilscale = powf(siteCount / (voro::optimal_particles * sizeX * sizeY * sizeZ), 1.0f / 3.0f);
		nx = int(sizeX * ilscale + 1);
		ny = int(sizeY * ilscale + 1);
		nz = int(sizeZ * ilscale + 1);
	}
}

namespace Riemann
{
	voro::container* CreateVoroContrainer(const std::vector<Vector3>& points, const Box3& _Bounds, const float eps)
	{
		const double BoundingBoxSlack = 0.01f;

		Box3 Bounds = _Bounds;
		Bounds = Bounds.Thicken(BoundingBoxSlack);
		Vector3 BoundingBoxSize = Bounds.GetSize();

		int GridCellsX, GridCellsY, GridCellsZ;
		voro::guess_optimal((int)points.size(), BoundingBoxSize.x, BoundingBoxSize.y, BoundingBoxSize.z, GridCellsX, GridCellsY, GridCellsZ);

		voro::container* Container = new voro::container(
			Bounds.Min.x, Bounds.Max.x, Bounds.Min.y, Bounds.Max.y, Bounds.Min.z, Bounds.Max.z,
			GridCellsX, GridCellsY, GridCellsZ, false, false, false, 10
			);

		if (eps > 0)
		{
			voro::voro_compute<voro::container> VoroCompute = Container->make_compute();
			for (size_t i = 0; i < points.size(); ++i)
			{
				const Vector3& v = points[i];
				double EX, EY, EZ;
				int ExistingPtID;
				if (Container->find_voronoi_cell(v.x, v.y, v.z, EX, EY, EZ, ExistingPtID, VoroCompute))
				{
					double dx = v.x - EX;
					double dy = v.y - EY;
					double dz = v.z - EZ;
					if (dx * dx + dy * dy + dz * dz < eps)
					{
						continue;
					}
				}
				Container->put((int)i, v.x, v.y, v.z);
			}
		}
		else
		{
			for (size_t i = 0; i < points.size(); ++i)
			{
				const Vector3& v = points[i];
				Container->put((int)i, v.x, v.y, v.z);
			}
		}

		return Container;
	}

	Voronoi3::Voronoi3()
	{
	}

	Voronoi3::Voronoi3(const std::vector<Vector3>& points, const Box3& bounds, const float eps)
	{
		Set(points, bounds, eps);
	}

	Voronoi3::~Voronoi3()
	{
		if (mContainer)
		{
			delete mContainer;
			mContainer = nullptr;
		}
	}

	void Voronoi3::Set(const std::vector<Vector3>& points, const Box3& bounds, const float eps)
	{
		mPoints = points;
		mBounds = bounds;
		mBounds.Encapsulate(Box3(points.data(), points.size()));
		mBounds.Thicken(1e-3f);
		if (mContainer)
		{
			delete mContainer;
		}
		mContainer = CreateVoroContrainer(points, bounds, eps);
		mNumPoints = points.size();
	}

	void Voronoi3::ComputeAllCells(std::vector<Cell>& cells, bool paraller_build)
	{
		voro::voro_compute<voro::container> c = mContainer->make_compute();
		cells.resize(mNumPoints);

		voro::c_loop_all it(*mContainer);
		voro::voronoicell_neighbor v;

		if (it.start())
		{
			do
			{
				bool success = mContainer->compute_cell(v, it, c);
				if (success)
				{
					int id = it.pid();
					double x, y, z;
					it.pos(x, y, z);
					Voronoi3::Cell &cell = cells[id];
					v.get_cell_info(Maths::TVector3<double>(x, y, z), cell.Vertices, cell.Faces, cell.Neighbors, cell.Normals);
				}
			} while (it.inc());
		}
	}

	void Voronoi3::ComputeAllNeighbors(std::vector<std::vector<int>>& neighbors, bool exclude_bounds, bool paraller_build)
	{
		voro::voro_compute<voro::container> c = mContainer->make_compute();

		neighbors.resize(mNumPoints);

		voro::c_loop_all it(*mContainer);
		voro::voronoicell_neighbor v;

		if (it.start())
		{
			do
			{
				bool success = mContainer->compute_cell(v, it, c);
				if (success)
				{
					int id = it.pid();

					v.neighborsTArray(neighbors[id], exclude_bounds);
				}
			} while (it.inc());
		}
	}

	void Voronoi3::ComputeAllEdges(std::vector<Edge>& edges, std::vector<int>& members, bool paraller_build)
	{
		std::vector<Vector3> Vertices;
		std::vector<int> FacesVertices;

		voro::c_loop_all it(*mContainer);
		voro::voronoicell cell;
		voro::voro_compute<voro::container> c = mContainer->make_compute();

		if (it.start())
		{
			if (!it.start())
			{
				return;
			}
			do
			{
				bool success = mContainer->compute_cell(cell, it, c);
				if (success)
				{
					int ID = it.pid();
					double X, Y, Z;
					it.pos(X, Y, Z);
					cell.get_cell_info(Maths::TVector3<double>(X, Y, Z), Vertices, FacesVertices);

					int FaceOffset = 0;
					for (size_t ii = 0, ni = FacesVertices.size(); ii < ni; ii += FacesVertices[ii] + 1)
					{
						int VertCount = FacesVertices[ii];
						int PreviousVertexIndex = FacesVertices[FaceOffset + VertCount];
						for (int kk = 0; kk < VertCount; ++kk)
						{
							members.push_back(ID);
							int VertexIndex = FacesVertices[1 + FaceOffset + kk]; // Index of vertex X coordinate in raw coordinates array

							edges.emplace_back(Vertices[PreviousVertexIndex], Vertices[VertexIndex]);
							PreviousVertexIndex = VertexIndex;
						}
						FaceOffset += VertCount + 1;
					}
				}
			} while (it.inc());
		}
	}

	void Voronoi3::GenerateRandomPoints(const Box3& Bounds, int numPoints, std::vector<Vector3>& points)
	{
		const Vector3 Extent(Bounds.Max - Bounds.Min);

		points.resize(numPoints);
		for (int i = 0; i < numPoints; ++i)
		{
			points[i] = Bounds.Min + Vector3::Random() * Extent;
		}
	}

	Box3 Voronoi3::GetVoronoiBounds(const Box3& Bounds, const std::vector<Vector3>& sites)
	{
		Box3 VoronoiBounds = Bounds;

		if (sites.size() > 0)
		{
			VoronoiBounds += Box3(sites.data(), sites.size());
		}

		return VoronoiBounds.Thicken(1e-3f);
	}

	void Voronoi3::Build()
	{
		mPlanes.clear();
		mCells.clear();
		mBoundaries.clear();
		mBoundaryVertices.clear();

		std::vector<Voronoi3::Cell> VoronoiCells;
		ComputeAllCells(VoronoiCells, true);

		int num_cells = (int)VoronoiCells.size();
		for (int i0 = 0; i0 < num_cells; ++i0)
		{
			int base = -1;

			const Voronoi3::Cell& cell = VoronoiCells[i0];
			int k = 0;
			for (size_t j = 0; j < cell.Neighbors.size(); ++j, k += 1 + cell.Faces[k])
			{
				int i1 = cell.Neighbors[j];
				if (i0 < i1)  // Filter out faces that we expect to get by symmetry
				{
					continue;
				}

				Vector3 Normal = cell.Normals[j];
				if (Normal.IsZero())
				{
					if (i1 > -1)
					{
						Normal = mPoints[i1] - mPoints[i0];
						Normal.Normalize();
					}
					else
					{
						continue;
					}
				}
				Plane P{ Normal.x, Normal.y, Normal.z, Normal.Dot(cell.Vertices[cell.Faces[k + 1]]) };
				if (base < 0)
				{
					base = (int)mBoundaryVertices.size();
					mBoundaryVertices.insert(mBoundaryVertices.end(), cell.Vertices.begin(), cell.Vertices.end());
				}
				std::vector<int> PlaneBoundary;
				int FaceSize = cell.Faces[k];
				for (int f = 0; f < FaceSize; f++)
				{
					PlaneBoundary.push_back(base + cell.Faces[k + 1 + f]);
				}

				mPlanes.push_back(P);
				mCells.emplace_back(i0, i1);
				mBoundaries.push_back(PlaneBoundary);
			}
		}
	}


}
