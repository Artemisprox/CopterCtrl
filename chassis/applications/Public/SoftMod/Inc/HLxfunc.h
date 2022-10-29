#ifndef __HLXFUNC_H__
#define __HLXFUNC_H__
/*移植自linux*/
/*使用时记得开启gnu扩展，lfp*/

/*非移植宏*/
#define Lx_PI         (3.1415926f)    //π


/*----------------------------------------绝对值函数--------------------------------------------*/
/**
 * abs - return absolute value of an argument
 * @x: the value.  If it is unsigned type, it is converted to signed type first.
 *     char is treated as if it was signed (regardless of whether it really is)
 *     but the macro's return type is preserved as char.
 *
 * Return: an absolute value of x.
 */

/*整型绝对值计算*/
#define Lx_abs(x)	__abs_choose_expr(x, long long,				\
		__abs_choose_expr(x, long,				\
		__abs_choose_expr(x, int,				\
		__abs_choose_expr(x, short,				\
		__abs_choose_expr(x, char,				\
		__builtin_choose_expr(					\
			__builtin_types_compatible_p(typeof(x), char),	\
			(char)({ signed char __x = (x); __x<0?-__x:__x; }), \
			((void)0)))))))//最后三行为迭代的起点

//当x为signed 或unsigned type数据类型时，进行绝对值比较，否则传递other
#define __abs_choose_expr(x, type, other) __builtin_choose_expr(	\
	__builtin_types_compatible_p(typeof(x),   signed type) ||	\
	__builtin_types_compatible_p(typeof(x), unsigned type),		\
	({ signed type __x = (x); __x < 0 ? -__x : __x; }), other)

/*浮点型绝对值计算,非移植宏*/
#define Lx_fabs(x) __fabs_choose_expr(x, double,				\
        __fabs_choose_expr(x, float, (void)0))

/*通用绝对值计算,非移植宏*/
#define Lx_Genabs(x) __fabs_choose_expr(x, double,				\
        __fabs_choose_expr(x, float,				\
        Lx_abs(x)))
/*非移植宏*/
#define __fabs_choose_expr(x, type, other) __builtin_choose_expr(   \
    __builtin_types_compatible_p(typeof(x),type),               \
    ({ type __x = (x); __x < 0 ? -__x : __x; }) , other)
/*-----------------------------------------------------------------------------------------------*/

/*----------------------------------------最大最小值函数------------------------------------------*/
//解析详见https://www.sohu.com/a/342675369_467784
/*
 * This returns a constant expression while determining if an argument is
 * a constant expression, most importantly without evaluating the argument.
 * Glory to Martin Uecker <Martin.Uecker@med.uni-goettingen.de>
 */
#define __is_constexpr(x) \
	(sizeof(int) == sizeof(*(8 ? ((void *)((long)(x) * 0l)) : (int *)8)))

/* Indirect macros required for expanded argument pasting, eg. __LINE__. */
#define ___PASTE(a,b) a##b
#define __LX_PASTE(a,b) ___PASTE(a,b)

#define __UNIQUE_ID(prefix) __LX_PASTE(__LX_PASTE(__UNIQUE_ID_, prefix), __COUNTER__)

/*
 * min()/max()/clamp() macros must accomplish three things:
 *
 * - avoid multiple evaluations of the arguments (so side-effects like
 *   "x++" happen only once) when non-constant.
 * - perform strict type-checking (to generate warnings instead of
 *   nasty runtime surprises). See the "unnecessary" pointer comparison
 *   in __typecheck().
 * - retain result as a constant expressions when called with only
 *   constant expressions (to avoid tripping VLA warnings in stack
 *   allocation usage).
 */
/* 如果他们是不同类型：编译有警告，表达式结果是 1
 * 如果他们是相同类型：一切风平浪静 ，表达式结果是 1*/
#define __typecheck(x, y) \
	(!!(sizeof((typeof(x) *)1 == (typeof(y) *)1)))

#define __no_side_effects(x, y) \
		(__is_constexpr(x) && __is_constexpr(y))

#define __safe_cmp(x, y) \
		(__typecheck(x, y) && __no_side_effects(x, y))

#define __cmp(x, y, op)	((x) op (y) ? (x) : (y))

#define __cmp_once(x, y, unique_x, unique_y, op) ({	\
		typeof(x) unique_x = (x);		\
		typeof(y) unique_y = (y);		\
		__cmp(unique_x, unique_y, op); })

#define __careful_cmp(x, y, op) \
	__builtin_choose_expr(__safe_cmp(x, y), \
		__cmp(x, y, op), \
		__cmp_once(x, y, __UNIQUE_ID(__x), __UNIQUE_ID(__y), op))

/**
 * Lx_min - return minimum of two values of the same or compatible types
 * @x: first value
 * @y: second value
 */
#define Lx_min(x, y)	__careful_cmp(x, y, <)

/**
 * Lx_max - return maximum of two values of the same or compatible types
 * @x: first value
 * @y: second value
 */
#define Lx_max(x, y)	__careful_cmp(x, y, >)

/**
 * Lx_min3 - return minimum of three values
 * @x: first value
 * @y: second value
 * @z: third value
 */
#define Lx_min3(x, y, z) Lx_min((typeof(x))Lx_min(x, y), z)

/**
 * Lx_max3 - return maximum of three values
 * @x: first value
 * @y: second value
 * @z: third value
 */
#define Lx_max3(x, y, z) Lx_max((typeof(x))Lx_max(x, y), z)

/**
 * Lx_max4 - return maximum of four values
 * @x: first value
 * @y: second value
 * @z: third value
 * @k: forth value
 */
#define Lx_max4(x, y, z, k) Lx_max((typeof(x))Lx_max3(x, y, z), k)/*非移植宏*/

/**
 * min_not_zero - return the minimum that is _not_ zero, unless both are zero
 * @x: value1
 * @y: value2
 */
#define Lx_min_not_zero(x, y) ({			\
	typeof(x) __x = (x);			\
	typeof(y) __y = (y);			\
	__x == 0 ? __y : ((__y == 0) ? __x : Lx_min(__x, __y)); })

/**
 * Lx_clamp - return a value clamped to a given range with strict typechecking
 * @val: current value
 * @lo: lowest allowable value
 * @hi: highest allowable value
 *
 * This macro does strict typechecking of @lo/@hi to make sure they are of the
 * same type as @val.  See the unnecessary pointer comparisons.
 */
#define Lx_clamp(val, lo, hi) Lx_min((typeof(val))Lx_max(val, lo), hi)

/*
 * ..and if you can't take the strict
 * types, you can specify one yourself.
 *
 * Or not use min/max/clamp at all, of course.
 */

/**
 * Lx_min_t - return minimum of two values, using the specified type
 * @type: val type to use
 * @x: first value
 * @y: second value
 */
#define Lx_min_t(type, x, y)	__careful_cmp((type)(x), (type)(y), <)

/**
 * Lx_max_t - return maximum of two values, using the specified type
 * @type: val type to use
 * @x: first value
 * @y: second value
 */
#define Lx_max_t(type, x, y)	__careful_cmp((type)(x), (type)(y), >)
/*-----------------------------------------------------------------------------------------------*/

/*------------------------------------------其他函数---------------------------------------------*/
/**
 * Lx_clamp_t - return a value clamped to a given range using a given type
 * @type: the type of variable to use
 * @val: current value
 * @lo: minimum allowable value
 * @hi: maximum allowable value
 *
 * This macro does no typechecking and uses temporary variables of type
 * @type to make all the comparisons.
 */
#define Lx_clamp_t(type, val, lo, hi) Lx_min_t(type, Lx_max_t(type, val, lo), hi)

/**
 * Lx_clamp_val - return a value clamped to a given range using val's type
 * @val: current value
 * @lo: minimum allowable value
 * @hi: maximum allowable value
 *
 * This macro does no typechecking and uses temporary variables of whatever
 * type the input argument @val is.  This is useful when @val is an unsigned
 * type and @lo and @hi are literals that will otherwise be assigned a signed
 * integer type.
 */
#define Lx_clamp_val(val, lo, hi) Lx_clamp_t(typeof(val), val, lo, hi)

/**
 * LX_MIRROR_CLAMP - 镜像范围钳位
 * @param val 	被限制值，注：不能是无符号的数据类型！！！
 * @param lmax 	允许的最大值，必须>0
 * @return 限制后数据,数据类型为val的数据类型
 */
#define LX_MIRROR_CLAMP(val,lmax) ({			\
    typeof(val) _val = val;          			\
    typeof(val) _lmax = lmax;          			\
	Lx_clamp_t(typeof(val),_val,-_lmax,_lmax);	\
})/*非移植宏*/

/**
 * Lx_swap - swap values of @a and @b
 * @a: first value
 * @b: second value
 */
#define Lx_swap(a, b) \
	do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)
 

/**
 * @brief   循环限制宏，限制val在(lmin~lmax]内，FIXME:算法上有比较大的优化空间，比如先整除再减的方式，O(1)的时间复杂度
 * @note	CIRCLE_CLAMP(22,-10,10) 返回2 ；CIRCLE_CLAMP(-32.5,0,5) 返回2.5
 * @param   val  被限制数据。当val为无符号数时，lmin，lmax必须>0
 * @param   lmin/lmax  最小/大值
 * @return	限制后数据,数据类型为val的数据类型
 */
#define CIRCLE_CLAMP(val,lmin,lmax)  ({     \
    typeof(val) _val = val;          		\
    typeof(val) _lmin = lmin;          		\
    typeof(val) _lmax = lmax;          		\
                                        	\
	if(_lmax > _lmin)						\
	{										\
    	if (_val > _lmax)                   \
    	{                                   \
    	    do                              \
    	    {                               \
    	        _val -= _lmax - _lmin;      \
    	    } while (_val > _lmax);         \
    	}                                   \
    	else if (_val <= _lmin)             \
    	{                                   \
    	    do                              \
    	    {                               \
    	        _val += _lmax - _lmin;      \
    	    } while (_val <= _lmin);        \
    	}                                   \
	}										\
    _val;                              		\
})/*非移植宏*/

/**
 * @brief   循环限制宏，限制data在(-limit~limit]内
 * @param   data  被限制数据，注：不能是无符号的数据类型！！！
 * @param   limit  最大值, 必须>0
 * @return	限制后数据，数据类型为data的数据类型
 */
#define CIRCLE_MIRROR_CLAMP(data,limit)  ({ \
    typeof(data) _data = data;          	\
    typeof(data) _limit = limit;          	\
	CIRCLE_CLAMP(_data,-_limit,_limit);		\
})/*非移植宏*/

/**
 * @brief   获取循环值之间较短的距离
 * @param start 转动起点数值，要在circle_len的范围内，
 * @param end   要转到的位置对应的数值，
 * @param circle_len 位置数据范围对应的整圈数值，>0，注：不能是无符号的数据类型！！！
 * @return  输出从start到end需要的最短路径的距离长度, 范围 (负半圈~正半圈]，正好半圈时取正半圈，数据类型为circle_len的数据类型
 */
#define CIRCLE_SHORTER_DIS(start,end,circle_len)  ({    \
    typeof(circle_len) _len = circle_len;               \
    typeof(_len) _start = start;                        \
    typeof(_len) _end = end;                            \
	typeof(_len) _dis,_HalfLen;                         \
	_dis = _end - _start;                               \
	_HalfLen = _len / 2;                                \
	if (_dis >= 0)                                      \
	{                                                   \
		if (_dis > _HalfLen)                            \
			_dis -= _len;                               \
	}                                                   \
	else                                                \
	{                                                   \
		if (_dis <= -_HalfLen)                          \
			_dis += _len;                               \
	}                                                   \
	_dis;                                               \
})/*非移植宏*/

/**
 * @brief   获取循环值之间最短的距离
 * @note	FIXME:有时间可以参考上面abs对于无符号类型数的强制转换;FIXME:可以参考上面实现自动防重命名的操作
 * @param start 转动起点数值，无限制范围；类型无限制
 * @param end   要转到的位置对应的数值，无限制范围；类型无限制
 * @param lo 	位置数据范围的下界，注：不能是无符号的数据类型！！！
 * @param hi 	位置数据范围的上界，类型必须和lo相同
 * @return  输出从start到end需要的最短路径的距离长度, 相对于起点的范围 (负半圈~正半圈]，正好半圈时取正半圈（正负方向和起点、终点的相同）
 */
#define CIRCLE_SHORTEST_DIS(start,end,lo,hi)  ({   		\
	if(__typecheck(lo,hi));								\
	typeof(hi) __start = start;							\
	typeof(hi) __end = end;								\
	__start = CIRCLE_CLAMP(__start,lo,hi); 				\
	__end = CIRCLE_CLAMP(__end,lo,hi); 					\
	CIRCLE_SHORTER_DIS(__start,__end,hi-lo); 			\
})/*非移植宏*/


/*时间探针结构体*/
typedef struct
{
	rt_tick_t	last_tick;
	rt_bool_t	if_init;
} Tick_probe_t;

#define TICK_PROBE_STORAGE(name)	static Tick_probe_t name = {0,RT_FALSE}
/**
 * @brief   获取所在函数被调用的时间间隔
 * @param record_p 静态的或者全局的变量指针
 * @return  当前被调用到上一次被调用的时间间隔，单位ms，float类型
 */
#define RT_TICK_PROBE(record_p)  ({											\
	Tick_probe_t *__record = record_p;										\
	float __period;															\
	if(__record->if_init == RT_TRUE)										\
    {																		\
        rt_tick_t now_tick = rt_tick_get();								    \
        __period = (float)(now_tick - __record->last_tick) 					\
					/(float)RT_TICK_PER_SECOND *1000.0f;/*时间间隔，单位ms*/ \
        __record->last_tick = now_tick;										\
    }																		\
    else/*第一次执行到,给last_tick赋初值*/									  \
    {																		\
        __record->last_tick = rt_tick_get();								\
        __record->if_init = RT_TRUE;										\
        __period = 0;														\
    }																		\
	__period;																\
})
/*-----------------------------------------------------------------------------------------------*/


#endif

