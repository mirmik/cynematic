#ifndef LINALG_ADDON_H
#define LINALG_ADDON_H

#include <linalg.h>
#include <linalg-ext.h>

namespace linalg
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

	template<typename T, int M>
	struct uvec : public vec<T,M> {
		uvec(const vec<T,M>& v) : vec<T,M>(normalize(v)) {}
		uvec(const uvec<T,M>& u) : vec<T,M>(u) {}
	};

	template<typename T>
	struct mtrans
	{
		quat<T> 	orient;
		vec<T, 3> 	center;

		mtrans() : orient{0,0,0,1}, center{0,0,0} {}
		mtrans(const quat<T>& r, const vec<T, 3> m) : orient(r), center(m) {}
		mtrans(const mat<T,3,3>& r, const vec<T, 3> m) : orient(rotation_quat(r)), center(m) {}

		mtrans operator*(const mtrans& oth) const
		{
			return mtrans(orient * oth.orient, qrot(orient, oth.center) + center);
		}

		mat<T, 4, 4> matrix() const
		{
			auto rmat = qmat(orient);
			return {{qxdir(orient), 0}, {qydir(orient), 0}, {qzdir(orient), 0}, {center, 1}};
		}

		vec<T, 6> vector6() const
		{
			auto angle = qangle(orient);
			auto rotvec = angle == 0 ? vec<T,3>{0,0,0} : qaxis(orient) * angle;
			return { rotvec[0], rotvec[1], rotvec[2], center[0], center[1], center[2] };
		}

		static mtrans rotation(uvec<T, 3> axis, T angle) { return mtrans(rotation_quat(axis, angle), vec<T, 3>()); }
		static mtrans translation(vec<T, 3> axis) { return mtrans(quat<T>(identity), axis); }

		vec<T, 3> rotate (const vec<T, 3>& v) const { return qrot(orient, v); }
		vec<T, 3> transform (const vec<T, 3>& v) const { return qrot(orient, v) + center; }

		auto vector6_to(const mtrans& target) const {
			return (target * inverse()).vector6();
		} 

		mtrans inverse() const { auto q = conjugate(orient); return { q, qrot(q, -center) }; }
	};

	template <typename T> mtrans<T> identity_lerp(const mtrans<T>& tr, T mul) { return { rotation_quat(qaxis(tr.orient), mul * qangle(tr.orient)), mul * tr.center }; }

	template <typename T> mtrans<T> lerp(const mtrans<T>& atr, const mtrans<T>& btr, T mul) { return atr * identity_lerp(atr.inverse() * btr, mul);  }

	/*struct mtrans
	{
		mat<T, 3, 3> orient;
		vec<T, 3> 	 center;

		mtrans() : orient {identity}, center {0,0,0} {}
		mtrans(const quat<T>& r, const vec<T, 3> m) : orient(qmat(r)), center(m) {}
		mtrans(const mat<T,3,3>& r, const vec<T, 3> m) : orient(r), center(m) {}

		mtrans operator*(const mtrans& oth)
		{
			return mtrans(orient * oth.orient, orient * oth.center + center);
		}

		mat<T, 4, 4> matrix()
		{
			return { {orient[0], 0}, {orient[1], 0}, {orient[2], 0}, {center, 1} };
		}

		/*vec<T, 6> vector6()
		{
			auto angle = qangle(orient);
			auto rotvec = angle == 0 ? vec<T,3>{0,0,0} : qaxis(orient) * angle;
			return { rotvec[0], rotvec[1], rotvec[2], center[0], center[1], center[2] };
		}*/

	/*	static mtrans rotation(vec<T, 3> axis, T angle) { return mtrans(rotation_quat(axis, angle), vec<T, 3>()); }
		static mtrans translation(vec<T, 3> axis) { return mtrans(quat<T>(identity), axis); }

		vec<T, 3> rotate (const vec<T, 3>& v) { return qrot(orient, v); }
		vec<T, 3> transform (const vec<T, 3>& v) { return qrot(orient, v) + center; }

		mtrans inverse() const { auto q = conjugate(orient); return { q, qrot(q, -center) }; }
	};*/

	template<class T>
	bivec<T, 3> speed_trans(const bivec<T, 3>& bispeed, const mtrans<T>& link)
	{
		auto w = bispeed.a;
		auto v = bispeed.b;

		auto trmat = link.inverse();

		auto res_w = trmat.rotate(w);
		auto res_v = trmat.rotate(cross(w, link.center) + v);

		return { res_w, res_v };
	}

	namespace ostream_overloads
    {
    	template<class C, class T> std::basic_ostream<C> & operator << (std::basic_ostream<C> & out, const mtrans<T> & tr) { return out << '{' << tr.orient << ',' << tr.center << '}'; }
    	template<class C, class T> std::basic_ostream<C> & operator << (std::basic_ostream<C> & out, const bivec<T,3> & tr) { return out << '{' << tr.a << ',' << tr.b << '}'; }
    }
}

#endif