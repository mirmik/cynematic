#ifndef LINALG_ADDON_H
#define LINALG_ADDON_H

#include <linalg.h>
#include <linalg-ext.h>

namespace cynematic 
{
	using namespace linalg;
	using namespace linalg::aliases;
	using namespace linalg::ostream_overloads;

	template<typename T, int N>
	struct bivec
	{
		vec<T, N> a;
		vec<T, N> b;

		bivec(const vec<T, N> _a, const vec<T, N> _b) : a(_a), b(_b) {};

		vec<T, 2 * N> concat()
		{
			vec<T, 2 * N> ret;

			for (int i = 0; i < N; ++i)
			{
				ret[i] = a[i];
				ret[i + N] = b[i];
			}

			return ret;
		}
	};

	template<typename T>
	struct mtrans
	{
		quat<T> 	rot;
		vec<T, 3> 	mov;

		mtrans() : rot{0,0,0,1}, mov{0,0,0} {}
		mtrans(const quat<T>& r, const vec<T, 3> m) : rot(r), mov(m) {}

		mtrans operator*(const mtrans& oth)
		{
			return mtrans(rot * oth.rot, qrot(rot, oth.mov) + mov);
		}

		mat<T, 4, 4> matrix()
		{
			auto rmat = qmat(rot);
			return {{qxdir(rot), 0}, {qydir(rot), 0}, {qzdir(rot), 0}, {mov, 1}};
		}

		vec<T, 6> vector6()
		{
			auto rotvec = qaxis(rot) * qangle(rot);
			return { rotvec[0], rotvec[1], rotvec[2], mov[0], mov[1], mov[2] };
		}

		static mtrans rotation(vec<T, 3> axis, T angle) { return mtrans(rotation_quat(axis, angle), vec<T, 3>()); }
		static mtrans translation(vec<T, 3> axis) { return mtrans(quat<T>(identity), axis); }

		vec<T, 3> rotate (const vec<T, 3>& v) { return qrot(rot, v); }
		vec<T, 3> transform (const vec<T, 3>& v) { return qrot(rot, v) + mov; }

		mtrans inverse() const { auto q = conjugate(rot); return { q, qrot(q, -mov) }; }
	};  

	template<class T>
	bivec<T, 3> speed_trans(const bivec<T, 3>& bispeed, const mtrans<T>& link)
	{
		auto w = bispeed.a;
		auto v = bispeed.b;

		auto trmat = link.inverse();

		auto res_w = trmat.rotate(w);
		auto res_v = trmat.rotate(cross(w, link.mov) + v);

		return { res_w, res_v };
	}
}

#endif