#ifndef CYNEMATIC_LINK_H
#define CYNEMATIC_LINK_H

#include <linalg.h>
#include <linalg-ext.h>

#include <nos/trace.h>

#include <initializer_list>

namespace cynematic
{
	using namespace linalg;
	using namespace linalg::aliases;
	using namespace linalg::ostream_overloads;

	template <typename T>
	struct abstract_link
	{
		using mat_t = linalg::mat<T, 4, 4>;
		using ax_t = linalg::vec<T, 3>;

		virtual mat_t get(const std::vector<T>& coords,
		                  uint8_t pos) = 0;
	
		virtual mat_t sensmat() = 0;
	
		virtual uint8_t count_of_coords() = 0;
	};

	template <typename T> 
	struct onedof_link : public abstract_link<T>
	{
		using mat_t = typename abstract_link<T>::mat_t;

		virtual mat_t get(T coord) = 0;

		mat_t get(const std::vector<T>& coords, uint8_t pos) override
		{
			return get(coords[pos]);
		}
	};

	template <typename T>
	struct constant_link : public abstract_link<T>
	{
		using mat_t = typename abstract_link<T>::mat_t;

		mat_t mat;

		mat_t get()
		{
			return mat;
		}

		mat_t get(const std::vector<T>& coords, uint8_t pos) override
		{
			return mat;
		}

		mat_t sensmat()
		{
			return mat_t();
		}

		uint8_t count_of_coords() override
		{
			return 0;
		}

		constant_link(mat_t _mat) : mat(_mat) {};
	};

	/*template<typename T, typename F>
	struct one_dof_link_t : public abstract_link<T>
	{
		F func;

		mat_t get(const std::vector<double>& coords, uint8_t pos) override
		{
			return func(coords[pos]);
		}

		mat_t sensmat()
		{
			return mat_t();
		}

		uint8_t count_of_coords() override
		{
			return 1;
		}

		one_dof_link_t(F _func) : func(_func) {};
	};

	template <typename F>
	one_dof_link_t<F> one_dof_link(F func) { return one_dof_link_t<F>(func); }
*/
	template<typename T>
	struct rotation_link : public onedof_link<T>
	{
		using mat_t = linalg::mat<T,4,4>;
		using ax_t = linalg::vec<T,3>;
		ax_t axvec;

		rotation_link(ax_t _axvec) : axvec(_axvec) {}

		mat_t get(T coord) override
		{
			return homogeneous_transformation<T, 3>::rotation( rotation_quat(axvec, coord) );
		}

		mat_t  sensmat()   //return homogeneous_transformation<double, 3>::rotation( rotation_quat(axvec, ) );
		{
			NOT_IMPLEMENTED();
		}

		uint8_t count_of_coords() override
		{
			return 1;
		}
	};
/*
	struct parametric_translation_link : public abstract_link
	{
		ax_t axvec;

		parametric_translation_link(double3 _axvec) : axvec(_axvec) {}

		mat_t get(const std::vector<double>& coords, uint8_t pos) override
		{
			return homogeneous_transformation<double, 3>::translation( axvec * coords[pos] );
		}

		mat_t sensmat()
		{
			return homogeneous_transformation<double, 3>::translation( axvec ) - mat_t(identity);
		}

		uint8_t count_of_coords() override { return 1; }
	};
*/
	template <typename T>
	struct chain
	{
		using mat_t = linalg::mat<T,4,4>;

		chain(){}
		chain(std::initializer_list<abstract_link<T>*> lst) : links(lst) {}

		void add_link(abstract_link<T>* lnk) { links.push_back(lnk); }

		mat_t get(const std::vector<T>& coords)
		{
			mat_t result = identity;
			int8_t coord_pos = coords.size() - 1;

			for (int i = links.size() - 1; i >= 0; --i)
			{
				uint8_t count_of_coords = links[i]->count_of_coords();

				if (coord_pos - count_of_coords + 1 < 0)
					return mat_t();

				mat_t nmat = links[i]->get(coords, coord_pos);
				result = nmat * result;
				coord_pos -= count_of_coords;
			}

			return result;
		}

		/*std::vector<mat_t> sensivity_matrices(const std::vector<double>& coords)
		{
			TRACE();
			std::vector<mat_t> result;
			mat_t curtrans = identity;
			int8_t coord_pos = coords.size() - 1;

			for (int i = links.size() - 1; i >= 0; --i)
			{
				uint8_t count_of_coords = links[i]->count_of_coords();

				if (coord_pos - count_of_coords + 1 < 0)
					return std::vector<mat_t>();

				if (count_of_coords > 0)
				{
					if (count_of_coords == 1)
					{
						auto sensmat = links[i]->sensmat();
						result.emplace_back(curtrans * sensmat);
					}
					else
					{
						return std::vector<mat_t>();
					}
				}

				mat_t nmat = links[i]->get(coords, coord_pos);
				curtrans = nmat * curtrans;

				coord_pos -= count_of_coords;
			}

			return result;

		}

		std::vector<double> sensivity(std::vector<double> curcoords, mat_t target)
		{
			auto curmat = get(curcoords);
			auto needmat = target * inverse(curmat);

			auto sensmats = sensivity_matrices(curcoords);

			std::cout << "needmat: " << needmat << std::endl;
			for (auto m : sensmats)
			{
				std::cout << "smat: " << m << std::endl;
			}
		}*/
	private:
		std::vector<abstract_link<T> *> links;
	};
/*
	struct dynamic_chain : public chain
	{
		~dynamic_chain()
		{
			for (int i = 0; i < links.size(); ++i) delete links[i];
		}
	};*/
}

#endif