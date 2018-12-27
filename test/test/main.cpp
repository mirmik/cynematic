#include <linalg.h>
#include <linalg-ext.h>
#include <linalg-add.h>

#include <iostream>
#include <vector>

#include <cynematic/link.h>
#include <cynematic/linalg-addon.h>

#include <gxx/time/chronotimer.h>

using namespace linalg;
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;
using namespace cynematic::ostream_overloads;

using namespace cynematic;

int main ()
{
	cynematic::rotation_link<double> r0(double3(0,0,1));
	cynematic::constant_link<double> c0(mtrans<double>::translation(double3(0,0,5)));
	cynematic::translation_link<double> t0(double3(1,0,0));
	cynematic::translation_link<double> t1(double3(0,0,1));
	cynematic::rotation_link<double> r2(double3(0,0,1));

	cynematic::chain<double> chain { &c0, &r0, &t0, &t1, &r2 };
	
	auto target = mtrans<double>::rotation(double3(0,0,1), deg(45)) * mtrans<double>::translation(double3(0,10,10));

	//nos::println(target);

	std::vector<double> res = {0,0,0,0};

	PRINT(chain.get({0,0,0,0}));
	PRINT(chain.get({1,0,0,0}));
	PRINT(chain.get({0,1,0,0}));
	PRINT(chain.get({0,0,1,0}));
	PRINT(chain.get({0,0,0,1}));
	PRINT(chain.get({1,1,1,1}));


	gxx::chronotimer ctimer;
	for (int i = 0; i < 11; ++i) {
		nos::println("HEREE!!!");

		ctimer.start();
		auto next_target = lerp(chain.get({0,0,0,0}), target, double(0.1 + 0.1 * i));
		res = chain.solve_inverse_cynematic(next_target, {0,0,0,0}); 
		ctimer.stop();
		PRINT(ctimer.micros());

		PRINT(next_target);
		PRINT(res);
		PRINT(chain.get(res));
	}



/*	nos::println(rot0.orient);
	nos::println(rot1.orient);
	nos::println(rot2.orient);

	nos::println(rot2.vector6_to(rot1));
	nos::println(rot2.vector6_to(rot0));
*/

	/*auto res = chain.get_speed_transes({deg(45),10,10});
	std::vector<double3> sens;
	std::vector<vec<double,6>> sens6;

	for (auto& r : res) {
		sens.push_back(r.b);
		sens6.push_back(r.concat());
	}

	double3 target {0,10,20};
	mtrans<double> target6 ({0,0,0,1},{0,0,0});

	double3 current = chain.get({0,10,10}).mov;
	mtrans<double> current6 = chain.get({0,10,10});

	auto need_to_target = target - current;
	auto need_to_target6 = target6 * current6.inverse();

	auto backp = backpack(normalize(need_to_target), sens);
	auto backp6 = backpack(normalize(need_to_target6.vector6()), sens6);

	PRINT(need_to_target);
	vec<double,3> accum;
	for (int i = 0; i < backp.size(); ++i) {
		accum += sens[i] * backp[i];
		PRINT(sens[i]);
		PRINT(backp[i]);
	}

	PRINT(need_to_target6.vector6());
	vec<double,6> accum6;
	for (int i = 0; i < backp6.size(); ++i) {
		accum6 += sens6[i] * backp6[i];
		PRINT(sens6[i]);
		PRINT(backp6[i]);
	}

	/*vec<double,6> target {0,0,1,0,28,2};
	auto unit_target = linalg::normalize(target);
	
	vec<double,6> v0 {0,0,1,0,1,1};
	v0 = linalg::normalize(v0);
	
	vec<double,6> v1 {0,0,0,0,1,1};
	v1 = linalg::normalize(v1);

	vec<double,6> v2 {0,0,0,0,1,-1};
	v2 = linalg::normalize(v2);
	
	double kv0 = 0;
	double kv1 = 0;
	double kv2 = 0;

	auto flow = [&]() { return kv0 * v0 + kv1 * v1 + kv2 * v2; };
	auto print = [&]() { nos::println(unit_target - flow(), kv0, kv1, kv2); };

	auto correct0 = [&]() { return linalg::dot(unit_target - flow(), v0); };
	auto correct1 = [&]() { return linalg::dot(unit_target - flow(), v1); };
	auto correct2 = [&]() { return linalg::dot(unit_target - flow(), v2); };

	print();

	auto last = unit_target;
	do {
		last = unit_target - flow();
		
		auto k0 = correct0();
		kv0 += k0;

		auto k1 = correct1(); 
		kv1 += k1;

		auto k2 = correct2();
		kv2 += k2;

		print();
	} while(linalg::length(unit_target - flow() - last) > 0.000001);*/
} 