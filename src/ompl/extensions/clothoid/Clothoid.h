/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2014                                                      |
 |                                                                          |
 |         , __                 , __                                        |
 |        /|/  \               /|/  \                                       |
 |         | __/ _   ,_         | __/ _   ,_                                | 
 |         |   \|/  /  |  |   | |   \|/  /  |  |   |                        |
 |         |(__/|__/   |_/ \_/|/|(__/|__/   |_/ \_/|/                       |
 |                           /|                   /|                        |
 |                           \|                   \|                        |
 |                                                                          |
 |      Enrico Bertolazzi                                                   |
 |      Dipartimento di Ingegneria Industriale                              |
 |      Universita` degli Studi di Trento                                   |
 |      email: enrico.bertolazzi@unitn.it                                   |
 |                                                                          |
\*--------------------------------------------------------------------------*/

///
/// file: Clothoid.hh
///

#ifndef CLOTHOID_HH
#define CLOTHOID_HH

#include <vector>
#include <iostream>
//! Clothoid computations routine
namespace Clothoid {

  using std::vector ;

  typedef double valueType ;
  typedef int    indexType ;
  
  /*\
   |   _____                         _
   |  |  ___| __ ___  ___ _ __   ___| |
   |  | |_ | '__/ _ \/ __| '_ \ / _ \ |
   |  |  _|| | |  __/\__ \ | | |  __/ |
   |  |_|  |_|  \___||___/_| |_|\___|_|
  \*/
  //! Compute Fresnel integrals
  /*!
   * \f[ C(x) = \int_0^x \cos\left(\frac{\pi}{2}t^2\right) dt, \qquad
   *     S(x) = \int_0^x \sin\left(\frac{\pi}{2}t^2\right) dt \f]
   * \param x the input abscissa
   * \param S the value of \f$ S(x) \f$
   * \param C the value of \f$ C(x) \f$
   */
  void
  FresnelCS( valueType   x,
             valueType & C,
             valueType & S ) ;

  //! Compute Fresnel integrals and its derivatives
  /*!
   * \f[ C(x) = \int_0^x \cos\left(\frac{\pi}{2}t^2\right) dt, \qquad
   *     S(x) = \int_0^x \sin\left(\frac{\pi}{2}t^2\right) dt \f]
   * \param x the input abscissa
   * \param S S[0]=\f$ S(x) \f$, S[1]=\f$ S'(x) \f$, S[2]=\f$ S''(x) \f$
   * \param C C[0]=\f$ C(x) \f$, C[1]=\f$ C'(x) \f$, C[2]=\f$ C''(x) \f$
   */
  void
  FresnelCS( indexType nk,
             valueType x,
             valueType C[],
             valueType S[] ) ;

  /*! \brief Compute the Fresnel integrals
   * \f[ 
   *   \int_0^1 t^k \cos\left(a\frac{t^2}{2} + b t + c\right) dt,\qquad
   *   \int_0^1 t^k \sin\left(a\frac{t^2}{2} + b t + c\right) dt
   * \f]
   * \param nk   number of momentae to compute
   * \param a    parameter \f$ a \f$
   * \param b    parameter \f$ b \f$
   * \param c    parameter \f$ c \f$
   * \param intC cosine integrals,
   * \param intS sine integrals
   */
  void
  GeneralizedFresnelCS( indexType nk,
                        valueType a,
                        valueType b,
                        valueType c,
                        valueType intC[],
                        valueType intS[] ) ;

  /*! \brief Compute the Fresnel integrals
   * \f[ 
   *   \int_0^1 t^k \cos\left(a\frac{t^2}{2} + b t + c\right) dt,\qquad
   *   \int_0^1 t^k \sin\left(a\frac{t^2}{2} + b t + c\right) dt
   * \f]
   * \param a      parameter \f$ a \f$
   * \param b      parameter \f$ b \f$
   * \param c      parameter \f$ c \f$
   * \param intC   cosine integrals, 
   * \param intS   sine integrals
   */
  void
  GeneralizedFresnelCS( valueType   a,
                        valueType   b,
                        valueType   c,
                        valueType & intC,
                        valueType & intS ) ;
  
  /*\
   |    ____ _       _   _           _     _
   |   / ___| | ___ | |_| |__   ___ (_) __| |
   |  | |   | |/ _ \| __| '_ \ / _ \| |/ _` |
   |  | |___| | (_) | |_| | | | (_) | | (_| |
   |   \____|_|\___/ \__|_| |_|\___/|_|\__,_|
  \*/
  /*! \brief Compute the clothoid by Hemite data
   *
   * \param x0     initial x position            \f$ x_0      \f$
   * \param y0     initial y position            \f$ y_0      \f$
   * \param theta0 initial angle                 \f$ \theta_0 \f$
   * \param x1     final x position              \f$ x_1      \f$
   * \param y1     final y position              \f$ y_1      \f$
   * \param theta1 final angle                   \f$ \theta_1 \f$
   * \param k      computed curvature            \f$ K        \f$
   * \param dk     computed curvature derivative \f$ K'       \f$
   * \param L      computed length of the curve
   */
  indexType
  buildClothoid( valueType   x0,
                 valueType   y0,
                 valueType   theta0,
                 valueType   x1,
                 valueType   y1,
                 valueType   theta1,
                 valueType & k,
                 valueType & dk,
                 valueType & L ) ;

  indexType
  buildClothoid( valueType   x0,
                 valueType   y0,
                 valueType   theta0,
                 valueType   x1,
                 valueType   y1,
                 valueType   theta1,
                 valueType & k,
                 valueType & dk,
                 valueType & L,
                 valueType & k_1,
                 valueType & dk_1,
                 valueType & L_1,
                 valueType & k_2,
                 valueType & dk_2,
                 valueType & L_2 ) ;

  //! Compute Lommel function
  valueType
  LommelReduced( valueType mu, valueType nu, valueType z ) ;
  
  class ClothoidCurve ; // forward declaration

  /*\
   |   _____     _                   _      ____  ____
   |  |_   _| __(_) __ _ _ __   __ _| | ___|___ \|  _ \
   |    | || '__| |/ _` | '_ \ / _` | |/ _ \ __) | | | |
   |    | || |  | | (_| | | | | (_| | |  __// __/| |_| |
   |    |_||_|  |_|\__,_|_| |_|\__, |_|\___|_____|____/
   |                           |___/
  \*/
  //! \brief Class to manage Triangle for BB of clothoid curve
  class Triangle2D {

    valueType p1[2], p2[2], p3[2] ;

  public:

    Triangle2D( ) {
      p1[0] = p1[1] =
      p2[0] = p2[1] =
      p3[0] = p3[1] = 0 ;
    }

    Triangle2D( valueType x1, valueType y1,
                valueType x2, valueType y2,
                valueType x3, valueType y3 ) {
      p1[0] = x1; p1[1] = y1;
      p2[0] = x2; p2[1] = y2;
      p3[0] = x3; p3[1] = y3;
    }

    Triangle2D( valueType const _p1[2],
                valueType const _p2[2],
                valueType const _p3[2] ) {
      p1[0] = _p1[0] ; p1[1] = _p1[1] ;
      p2[0] = _p2[0] ; p2[1] = _p2[1] ;
      p3[0] = _p3[0] ; p3[1] = _p3[1] ;
    }

    ~Triangle2D() {}
    
    valueType x1() const { return p1[0] ; }
    valueType y1() const { return p1[1] ; }
    valueType x2() const { return p2[0] ; }
    valueType y2() const { return p2[1] ; }
    valueType x3() const { return p3[0] ; }
    valueType y3() const { return p3[1] ; }

    bool intersect( Triangle2D const & t2 ) const ;
    bool overlap( Triangle2D const & t2 ) const ;
    
    friend class ClothoidCurve ;

  };
  
  /*\
   |    ____ _       _   _           _     _  ____
   |   / ___| | ___ | |_| |__   ___ (_) __| |/ ___|   _ _ ____   _____
   |  | |   | |/ _ \| __| '_ \ / _ \| |/ _` | |  | | | | '__\ \ / / _ \
   |  | |___| | (_) | |_| | | | (_) | | (_| | |__| |_| | |   \ V /  __/
   |   \____|_|\___/ \__|_| |_|\___/|_|\__,_|\____\__,_|_|    \_/ \___|
  \*/
  //! \brief Class to manage Clothoid Curve
  class ClothoidCurve {

	public:
    valueType x0,       //!< initial x coordinate of the clothoid
              y0,       //!< initial y coordinate of the clothoid
              theta0 ;  //!< initial angle of the clothoid

    valueType k,        //!< initial curvature
              dk,       //!< curvature derivative
              s_min,    //!< initial curvilinear coordinate of the clothoid segment
              s_max ;   //!< final curvilinear coordinate of the clothoid segment

    void
    bbSplit_internal( valueType               split_angle,
                      valueType               split_size,
                      valueType               split_offs,
                      vector<ClothoidCurve> & c,
                      vector<Triangle2D>    & t ) const ;

    //! Use newton and bisection to intersect two small clothoid segment
    bool
    intersect_internal( ClothoidCurve & c1, valueType c1_offs, valueType & s1,
                        ClothoidCurve & c2, valueType c2_offs, valueType & s2,
                        indexType max_iter,
                        valueType tolerance ) const ;

  public:
  
    ClothoidCurve()
    : x0(0)
    , y0(0)
    , theta0(0)
    , k(0)
    , dk(0)
    , s_min(0)
    , s_max(0)
    {}

    //! construct a clothoid with the standard parameters
    ClothoidCurve( valueType _x0,
                   valueType _y0,
                   valueType _theta0,
                   valueType _k,
                   valueType _dk,
                   valueType _L )
    : x0(_x0)
    , y0(_y0)
    , theta0(_theta0)
    , k(_k)
    , dk(_dk)
    , s_min(0)
    , s_max(_L)
    {}

    ClothoidCurve( valueType _x0,
                   valueType _y0,
                   valueType _theta0,
                   valueType _k,
                   valueType _dk,
                   valueType _smin ,
                   valueType _smax )
    : x0(_x0)
    , y0(_y0)
    , theta0(_theta0)
    , k(_k)
    , dk(_dk)
    , s_min(_smin)
    , s_max(_smax)
    {}

    //! construct a clothoid by solving the hermite G1 problem
    ClothoidCurve( valueType const _P0[],
                   valueType       _theta0,
                   valueType const _P1[],
                   valueType       _theta1 )
    : x0(_P0[0])
    , y0(_P0[1])
    , theta0(_theta0)
    , s_min(0) {
      buildClothoid( x0, y0, theta0, _P1[0], _P1[1], _theta1, k, dk, s_max ) ;
    }

    void
    copy( ClothoidCurve const & c ) {
      x0     = c.x0 ;
      y0     = c.y0 ;
      theta0 = c.theta0 ;
      k      = c.k ;
      dk     = c.dk ;
      s_min  = c.s_min ;
      s_max  = c.s_max ;
    }

    ClothoidCurve( ClothoidCurve const & s ) { copy(s) ; }

    ClothoidCurve const & operator = ( ClothoidCurve const & s )
    { copy(s) ; return *this ; }
    
    valueType getX0()      const { return x0 ; }
    valueType getY0()      const { return y0 ; }
    valueType getTheta0()  const { return theta0 ; }
    valueType getKappa()   const { return k ; }
    valueType getKappa_D() const { return dk ; }
    valueType getSmin()    const { return s_min ; }
    valueType getSmax()    const { return s_max ; }

    //! construct a clothoid with the standard parameters
    void
    setup( valueType _x0,
           valueType _y0,
           valueType _theta0,
           valueType _k,
           valueType _dk,
           valueType _L ) {
      x0     = _x0 ;
      y0     = _y0 ;
      theta0 = _theta0 ;
      k      = _k ;
      dk     = _dk ;
      s_min  = 0 ;
      s_max  = _L ;
    }

    void
    setup( valueType _x0,
           valueType _y0,
           valueType _theta0,
           valueType _k,
           valueType _dk,
           valueType _smin,
           valueType _smax ) {
      x0     = _x0 ;
      y0     = _y0 ;
      theta0 = _theta0 ;
      k      = _k ;
      dk     = _dk ;
      s_min  = _smin ;
      s_max  = _smax ;
    }

    //! build a clothoid by solving the hermite G1 problem
    void
    setup_G1( valueType _x0,
              valueType _y0,
              valueType _theta0,
              valueType _x1,
              valueType _y1,
              valueType _theta1 ) {
      buildClothoid( _x0, _y0, _theta0, _x1, _y1, _theta1, k, dk, s_max ) ;
      x0     = _x0 ;
      y0     = _y0 ;
      theta0 = _theta0 ;
      s_min  = 0 ;
    }

    //! build a clothoid by solving the forward problem
    bool
    setup_forward( valueType _x0,
                   valueType _y0,
                   valueType _theta0,
                   valueType _k,
                   valueType _x1,
                   valueType _y1,
                   valueType tol = 1e-8 ) ;

    valueType
    theta( valueType s ) const { return theta0 + s*(k + 0.5*s*dk) ; }

    valueType
    theta_D( valueType s ) const { return k + s*dk ; }

    valueType
    theta_DD( valueType ) const { return dk ; }

    valueType
    theta_DDD( valueType ) const { return 0 ; }

    void
    eval( valueType   s,
          valueType & theta,
          valueType & kappa,
          valueType & x,
          valueType & y ) const ;

    void eval( valueType s, valueType & x, valueType & y ) const ;
    void eval_D( valueType s, valueType & x_D, valueType & y_D ) const ;
    void eval_DD( valueType s, valueType & x_DD, valueType & y_DD ) const ;
    void eval_DDD( valueType s, valueType & x_DDD, valueType & y_DDD ) const ;

    // offset curve
    void eval( valueType s, valueType offs, valueType & x, valueType & y ) const ;
    void eval_D( valueType s, valueType offs, valueType & x_D, valueType & y_D ) const ;
    void eval_DD( valueType s, valueType offs, valueType & x_DD, valueType & y_DD ) const ;
    void eval_DDD( valueType s, valueType offs, valueType & x_DDD, valueType & y_DDD ) const ;

    void
    trim( valueType s_begin, valueType s_end ) {
      s_min = s_begin ;
      s_max = s_end ;
    }

    //! set the origin of the clothoid to the curvilinear abscissa s0
    void change_origin( valueType s0 ) ;

    //! get the bounding box triangle (if angle variation less that pi/2)
    bool
    bbTriangle( valueType offs,
                valueType p0[2],
                valueType p1[2],
                valueType p2[2] ) const ;

    bool
    bbTriangle( valueType offs, Triangle2D & t ) const
    { return bbTriangle( offs, t.p1, t.p2, t.p3 ) ; }

    void
    bbSplit( valueType               split_angle, //!< maximum angle variation
             valueType               split_size,  //!< maximum height of the triangle
             valueType               split_offs,  //!< curve offset
             vector<ClothoidCurve> & c,           //!< clothoid segments
             vector<Triangle2D>    & t ) const ;  //!< clothoid bounding box

    // intersect computation
    void
    intersect( ClothoidCurve const & c,
               vector<valueType>   & s1,
               vector<valueType>   & s2,
               indexType             max_iter,
               valueType             tolerance ) const {
      intersect( 0, c, 0, s1, s2, max_iter, tolerance ) ;
    }

    void
    intersect( valueType             offs,
               ClothoidCurve const & c,
               valueType             c_offs,
               vector<valueType>   & s1,
               vector<valueType>   & s2,
               indexType             max_iter,
               valueType             tolerance ) const ;

    // collision detection
    bool
    approsimate_collision( valueType             offs,
                           ClothoidCurve const & c,
                           valueType             c_offs,
                           valueType             max_angle,         //!< maximum angle variation
                           valueType             max_size ) const ; //!< curve offset

    friend
    std::ostream &
    operator << ( std::ostream & stream, ClothoidCurve const & c ) ;

    void
    rotate( valueType angle, valueType cx, valueType cy ) ;

    void
    translate( valueType tx, valueType ty )
    { x0 += tx ; y0 += ty ; }

    void
    moveOrigin( valueType newx0, valueType newy0 )
    { x0 = newx0 ; y0 = newy0 ; }

    void
    scale( valueType s ) ;

    void
    reverse() ;

  } ;
  
  /*\
   |    ____ ____     _       _
   |   / ___|___ \ __| | __ _| |_ __ _
   |  | |  _  __) / _` |/ _` | __/ _` |
   |  | |_| |/ __/ (_| | (_| | || (_| |
   |   \____|_____\__,_|\__,_|\__\__,_|
  \*/
  class G2data {

  protected:

    valueType tolerance ;
    int       maxIter ;

    valueType x0 ;
    valueType y0 ;
    valueType theta0 ;
    valueType kappa0 ;
    valueType x1 ;
    valueType y1 ;
    valueType theta1 ;
    valueType kappa1 ;

    // standard problem
    valueType phi, Lscale ;
    valueType th0, th1 ;
    valueType k0, k1 ;
    valueType DeltaK ;
    valueType DeltaTheta ;

  public:
  
    G2data()
    : tolerance(1e-10)
    , maxIter(20)
    , x0(0)
    , y0(0)
    , theta0(0)
    , kappa0(0)
    , x1(0)
    , y1(0)
    , theta1(0)
    , kappa1(0)
    , phi(0)
    , Lscale(0)
    , th0(0)
    , th1(0)
    , k0(0)
    , k1(0)
    {}

    ~G2data() {}

    void
    setup( valueType x0, valueType y0, valueType theta0, valueType kappa0,
           valueType x1, valueType y1, valueType theta1, valueType kappa1 ) ;

    void
    setTolerance( valueType tol ) ;

    void
    setMaxIter( int tol ) ;

  } ;
  
  /*\
   |    ____ ____            _           ____
   |   / ___|___ \ ___  ___ | |_   _____|___ \ __ _ _ __ ___
   |  | |  _  __) / __|/ _ \| \ \ / / _ \ __) / _` | '__/ __|
   |  | |_| |/ __/\__ \ (_) | |\ V /  __// __/ (_| | | | (__
   |   \____|_____|___/\___/|_| \_/ \___|_____\__,_|_|  \___|
  \*/
  class G2solve2arc : public G2data {

    ClothoidCurve S0, S1 ;

    void
    evalA( valueType   alpha,
           valueType   L,
           valueType & A,
           valueType & A_1,
           valueType & A_2 ) const ;

    void
    evalG( valueType alpha,
           valueType L,
           valueType th,
           valueType k,
           valueType G[2],
           valueType G_1[2],
           valueType G_2[2] ) const ;

    void
    evalFJ( valueType const vars[2],
            valueType       F[2],
            valueType       J[2][2] ) const ;

    void
    buildSolution( valueType alpha, valueType L ) ;

  public:
  
    using G2data::setup ;
    using G2data::setTolerance ;
    using G2data::setMaxIter ;

    G2solve2arc() {}
    ~G2solve2arc() {}

    bool
    solve() ;

    ClothoidCurve const & getS0() const { return S0 ; }
    ClothoidCurve const & getS1() const { return S1 ; }

  } ;
  
  /*\
   |    ____ ____            _           _____
   |   / ___|___ \ ___  ___ | |_   _____|___ /  __ _ _ __ ___
   |  | |  _  __) / __|/ _ \| \ \ / / _ \ |_ \ / _` | '__/ __|
   |  | |_| |/ __/\__ \ (_) | |\ V /  __/___) | (_| | | | (__
   |   \____|_____|___/\___/|_| \_/ \___|____/ \__,_|_|  \___|
  \*/
  class G2solve3arc : public G2data {

    ClothoidCurve S0, SM, S1 ;

    valueType alpha, beta, gamma ;
    valueType dK0_0, dK0_1, dK0_2 ;
    valueType dK1_0, dK1_1, dK1_2 ;
    valueType KM_0,  KM_1,  KM_2  ;
    valueType thM_0, thM_1, thM_2 ;
    valueType a0, b1, gamma2 ;

    void
    evalFJ( valueType const vars[2],
            valueType       F[2],
            valueType       J[2][2] ) const ;

    void
    buildSolution( valueType eta, valueType zeta ) ;

  public:
    
    using G2data::setup ;
    using G2data::setTolerance ;
    using G2data::setMaxIter ;

    G2solve3arc() {}
    ~G2solve3arc() {}

    bool
    solve() ;

    void
    setup( valueType _x0,
           valueType _y0,
           valueType _theta0,
           valueType _kappa0,
           valueType _alpha,
           valueType _x1,
           valueType _y1,
           valueType _theta1,
           valueType _kappa1,
           valueType _beta ) ;

    ClothoidCurve const & getS0() const { return S0 ; }
    ClothoidCurve const & getS1() const { return S1 ; }
    ClothoidCurve const & getSM() const { return SM ; }

  } ;

}

#endif

///
/// eof: Clothoid.hh
///
