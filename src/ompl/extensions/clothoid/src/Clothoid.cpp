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

#include "../Clothoid.h"
#include "../CubicRootsFlocke.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

#ifndef CLOTHOID_ASSERT
  #define CLOTHOID_ASSERT(COND,MSG)         \
    if ( !(COND) ) {                        \
      std::ostringstream ost ;              \
      ost << "On line: " << __LINE__        \
          << " file: " << __FILE__          \
          << '\n' << MSG << '\n' ;          \
      throw std::runtime_error(ost.str()) ; \
    }
#endif

#define A_THRESOLD   0.01
#define A_SERIE_SIZE 3

namespace Clothoid {

  using namespace std ;

  /*
  // This function calculates the fresnel cosine and sine integrals.
  // Input:
  // y = values for which fresnel integrals have to be evaluated  
  //
  // Output:
  // FresnelC = fresnel cosine integral of y
  // FresnelS = fresnel sine integral of y  
  //
  // Adapted from:
  // Atlas for computing mathematical functions : an illustrated guide for
  // practitioners, with programs in C and Mathematica / William J. Thompson.
  // New York : Wiley, c1997.
  //
  // Author: Venkata Sivakanth Telasula
  // email: sivakanth.telasula@gmail.com
  // date: August 11, 2005
  */
  //! \cond NODOC
  static const valueType fn[] = { 0.49999988085884732562,
                                  1.3511177791210715095,
                                  1.3175407836168659241,
                                  1.1861149300293854992,
                                  0.7709627298888346769,
                                  0.4173874338787963957,
                                  0.19044202705272903923,
                                  0.06655998896627697537,
                                  0.022789258616785717418,
                                  0.0040116689358507943804,
                                  0.0012192036851249883877 } ;

  static const valueType fd[] = { 1.0,
                                  2.7022305772400260215,
                                  4.2059268151438492767,
                                  4.5221882840107715516,
                                  3.7240352281630359588,
                                  2.4589286254678152943,
                                  1.3125491629443702962,
                                  0.5997685720120932908,
                                  0.20907680750378849485,
                                  0.07159621634657901433,
                                  0.012602969513793714191,
                                  0.0038302423512931250065 } ;

  static const valueType gn[] = { 0.50000014392706344801,
                                  0.032346434925349128728,
                                  0.17619325157863254363,
                                  0.038606273170706486252,
                                  0.023693692309257725361,
                                  0.007092018516845033662,
                                  0.0012492123212412087428,
                                  0.00044023040894778468486,
                                 -8.80266827476172521e-6,
                                 -1.4033554916580018648e-8,
                                  2.3509221782155474353e-10 } ;

  static const valueType gd[] = { 1.0,
                                  2.0646987497019598937,
                                  2.9109311766948031235,
                                  2.6561936751333032911,
                                  2.0195563983177268073,
                                  1.1167891129189363902,
                                  0.57267874755973172715,
                                  0.19408481169593070798,
                                  0.07634808341431248904,
                                  0.011573247407207865977,
                                  0.0044099273693067311209,
                                 -0.00009070958410429993314 } ;

  static const valueType m_pi        = 3.14159265358979323846264338328  ; // pi
  static const valueType m_pi_2      = 1.57079632679489661923132169164  ; // pi/2
  static const valueType m_2pi       = 6.28318530717958647692528676656  ; // 2*pi
  static const valueType m_1_pi      = 0.318309886183790671537767526745 ; // 1/pi
  static const valueType m_1_sqrt_pi = 0.564189583547756286948079451561 ; // 1/sqrt(pi)

  //! \endcond

  /*
  //  #######                                           
  //  #       #####  ######  ####  #    # ###### #      
  //  #       #    # #      #      ##   # #      #      
  //  #####   #    # #####   ####  # #  # #####  #      
  //  #       #####  #           # #  # # #      #      
  //  #       #   #  #      #    # #   ## #      #      
  //  #       #    # ######  ####  #    # ###### ###### 
  */

  void
  FresnelCS( valueType y, valueType & C, valueType & S ) {
    /*=======================================================*\
      Purpose: This program computes the Fresnel integrals 
               C(x) and S(x) using subroutine FCS
      Input :  x --- Argument of C(x) and S(x)
      Output:  C --- C(x)
               S --- S(x)
      Example:
              x          C(x)          S(x)
             -----------------------------------
             0.0      .00000000      .00000000
             0.5      .49234423      .06473243
             1.0      .77989340      .43825915
             1.5      .44526118      .69750496
             2.0      .48825341      .34341568
             2.5      .45741301      .61918176

      Purpose: Compute Fresnel integrals C(x) and S(x)
      Input :  x --- Argument of C(x) and S(x)
      Output:  C --- C(x)
               S --- S(x)
    \*=======================================================*/

    valueType const eps = 1E-15 ;    
    valueType const x   = y > 0 ? y : -y ;

    if ( x < 1.0 ) {
      valueType twofn, fact, denterm, numterm, sum, term ;

      valueType const s = m_pi_2*(x*x) ;
      valueType const t = -s*s ;

      // Cosine integral series
      twofn   =  0.0 ;
      fact    =  1.0 ;
      denterm =  1.0 ;
      numterm =  1.0 ;
      sum     =  1.0 ;
      do {
        twofn   += 2.0 ;
        fact    *= twofn*(twofn-1.0);
        denterm += 4.0 ;
        numterm *= t ;
        term     = numterm/(fact*denterm) ;
        sum     += term ;
      } while ( std::abs(term) > eps*std::abs(sum) ) ;

      C = x*sum;

      // Sine integral series
      twofn   = 1.0 ;
      fact    = 1.0 ;
      denterm = 3.0 ;
      numterm = 1.0 ;
      sum     = 1.0/3.0 ;
      do {
        twofn   += 2.0 ;
        fact    *= twofn*(twofn-1.0) ;
        denterm += 4.0 ;
        numterm *= t ;
        term     = numterm/(fact*denterm) ;
        sum     += term ;
      } while ( std::abs(term) > eps*std::abs(sum) ) ;

      S = m_pi_2*sum*(x*x*x) ;

    } else if ( x < 6.0 ) {

      // Rational approximation for f
      valueType sumn = 0.0 ;
      valueType sumd = fd[11] ;
      for ( indexType k=10 ; k >= 0 ; --k ) {
        sumn = fn[k] + x*sumn ;
        sumd = fd[k] + x*sumd ;
      }
      valueType f = sumn/sumd ;

      // Rational approximation for g
      sumn = 0.0 ;
      sumd = gd[11] ;
      for ( indexType k=10 ; k >= 0 ; --k ) {
        sumn = gn[k] + x*sumn ;
        sumd = gd[k] + x*sumd ;
      }
      valueType g = sumn/sumd ;

      valueType U    = m_pi_2*(x*x) ;
      valueType SinU = sin(U) ;
      valueType CosU = cos(U) ;
      C = 0.5 + f*SinU - g*CosU ;
      S = 0.5 - f*CosU - g*SinU ;

    } else {

      valueType absterm ;

      // x >= 6; asymptotic expansions for  f  and  g

      valueType const s = m_pi*x*x ;
      valueType const t = -1/(s*s) ;

      // Expansion for f
      valueType numterm = -1.0 ;
      valueType term    =  1.0 ;
      valueType sum     =  1.0 ;
      valueType oldterm =  1.0 ;
      valueType eps10   =  0.1 * eps ;

      do {
        numterm += 4.0 ;
        term    *= numterm*(numterm-2.0)*t ;
        sum     += term ;
        absterm  = std::abs(term) ;
        CLOTHOID_ASSERT( oldterm >= absterm,
                         "In FresnelCS f not converged to eps, x = " << x <<
                         " oldterm = " << oldterm << " absterm = " << absterm ) ;
        oldterm  = absterm ;
      } while ( absterm > eps10 * std::abs(sum) ) ;

      valueType f = sum / (m_pi*x) ;

      //  Expansion for  g
      numterm = -1.0 ;
      term    =  1.0 ;
      sum     =  1.0 ;
      oldterm =  1.0 ;

      do {
        numterm += 4.0 ;
        term    *= numterm*(numterm+2.0)*t ;
        sum     += term ;
        absterm  = std::abs(term) ;
        CLOTHOID_ASSERT( oldterm >= absterm,
                         "In FresnelCS g not converged to eps, x = " << x <<
                         " oldterm = " << oldterm << " absterm = " << absterm ) ;
        oldterm  = absterm ;
      } while ( absterm > eps10 * std::abs(sum) ) ;

      valueType g = m_pi*x ; g = sum/(g*g*x) ;

      valueType U    = m_pi_2*(x*x) ;
      valueType SinU = sin(U) ;
      valueType CosU = cos(U) ;
      C = 0.5 + f*SinU - g*CosU ;
      S = 0.5 - f*CosU - g*SinU ;
      
    }
    if ( y < 0 ) { C = -C ; S = -S ; }
  }

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  void
  FresnelCS( indexType nk,
             valueType t,
             valueType C[],
             valueType S[] ) {
    FresnelCS(t,C[0],S[0]) ;
    if ( nk > 1 ) {
      valueType tt = m_pi_2*(t*t) ;
      valueType ss = sin(tt) ;
      valueType cc = cos(tt) ;
      C[1] = ss*m_1_pi ;
      S[1] = (1-cc)*m_1_pi ;
      if ( nk > 2 ) {
        C[2] = (t*ss-S[0])*m_1_pi ;
        S[2] = (C[0]-t*cc)*m_1_pi ;
      }
    }
  }

  //! \cond NODOC

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  static
  void
  evalXYaLarge( valueType   a,
                valueType   b,
                valueType & X,
                valueType & Y ) {
    valueType s    = a > 0 ? +1 : -1 ;
    valueType absa = std::abs(a) ;
    valueType z    = m_1_sqrt_pi*sqrt(absa) ;
    valueType ell  = s*b*m_1_sqrt_pi/sqrt(absa) ;
    valueType g    = -0.5*s*(b*b)/absa ;
    valueType cg   = cos(g)/z ;
    valueType sg   = sin(g)/z ;

    valueType Cl, Sl, Cz, Sz ;
    FresnelCS( ell,   Cl, Sl ) ;
    FresnelCS( ell+z, Cz, Sz ) ;

    valueType dC0 = Cz - Cl ;
    valueType dS0 = Sz - Sl ;

    X = cg * dC0 - s * sg * dS0 ;
    Y = sg * dC0 + s * cg * dS0 ;
  }

  // -------------------------------------------------------------------------
  // nk max 3
  static
  void
  evalXYaLarge( indexType nk,
                valueType a,
                valueType b,
                valueType X[],
                valueType Y[] ) {

    CLOTHOID_ASSERT( nk < 4 && nk > 0,
                     "In evalXYaLarge first argument nk must be in 1..3, nk " << nk ) ;

    valueType s    = a > 0 ? +1 : -1 ;
    valueType absa = std::abs(a) ;
    valueType z    = m_1_sqrt_pi*sqrt(absa) ;
    valueType ell  = s*b*m_1_sqrt_pi/sqrt(absa) ;
    valueType g    = -0.5*s*(b*b)/absa ;
    valueType cg   = cos(g)/z ;
    valueType sg   = sin(g)/z ;

    valueType Cl[3], Sl[3], Cz[3], Sz[3] ;

    FresnelCS( nk, ell,   Cl, Sl ) ;
    FresnelCS( nk, ell+z, Cz, Sz ) ;

    valueType dC0 = Cz[0] - Cl[0] ;
    valueType dS0 = Sz[0] - Sl[0] ;
    X[0] = cg * dC0 - s * sg * dS0 ;
    Y[0] = sg * dC0 + s * cg * dS0 ;
    if ( nk > 1 ) {
      cg /= z ;
      sg /= z ;
      valueType dC1 = Cz[1] - Cl[1] ;
      valueType dS1 = Sz[1] - Sl[1] ;
      valueType DC  = dC1-ell*dC0 ;
      valueType DS  = dS1-ell*dS0 ;
      X[1] = cg * DC - s * sg * DS ;
      Y[1] = sg * DC + s * cg * DS ;
      if ( nk > 2 ) {
        valueType dC2 = Cz[2] - Cl[2] ;
        valueType dS2 = Sz[2] - Sl[2] ;
        DC   = dC2+ell*(ell*dC0-2*dC1) ;
        DS   = dS2+ell*(ell*dS0-2*dS1) ;
        cg   = cg/z ;
        sg   = sg/z ;
        X[2] = cg * DC - s * sg * DS ;
        Y[2] = sg * DC + s * cg * DS ;
      }
    }
  }

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  valueType
  LommelReduced( valueType mu, valueType nu, valueType b ) {
    valueType tmp = 1/((mu+nu+1)*(mu-nu+1)) ;
    valueType res = tmp ;
    for ( indexType n = 1 ; n <= 100 ; ++n ) {
      tmp *= (-b/(2*n+mu-nu+1)) * (b/(2*n+mu+nu+1)) ;
      res += tmp ;
      if ( std::abs(tmp) < std::abs(res) * 1e-50 ) break ;
    }
    return res ;
  }

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  static
  void
  evalXYazero( indexType nk,
               valueType b,
               valueType X[],
               valueType Y[] ) {

    valueType sb = sin(b) ;
    valueType cb = cos(b) ;
    valueType b2 = b*b ;
    if ( std::abs(b) < 1e-3 ) {
      X[0] = 1-(b2/6)*(1-(b2/20)*(1-(b2/42))) ;
      Y[0] = (b/2)*(1-(b2/12)*(1-(b2/30))) ;
    } else {
      X[0] = sb/b ;
      Y[0] = (1-cb)/b ;
    }
    // use recurrence in the stable part
    indexType m = indexType(floor(2*b)) ;
    if ( m >= nk ) m = nk-1 ;
    if ( m < 1   ) m = 1 ;
    for ( indexType k = 1 ; k < m ; ++k ) {
      X[k] = (sb-k*Y[k-1])/b ;
      Y[k] = (k*X[k-1]-cb)/b ;
    }
    //  use Lommel for the unstable part
    if ( m < nk ) {
      valueType A   = b*sb ;
      valueType D   = sb-b*cb ;
      valueType B   = b*D ;
      valueType C   = -b2*sb ;
      valueType rLa = LommelReduced(m+0.5,1.5,b) ;
      valueType rLd = LommelReduced(m+0.5,0.5,b) ;
      for ( indexType k = m ; k < nk ; ++k ) {
        valueType rLb = LommelReduced(k+1.5,0.5,b) ;
        valueType rLc = LommelReduced(k+1.5,1.5,b) ;
        X[k] = ( k*A*rLa + B*rLb + cb ) / (1+k) ;
        Y[k] = ( C*rLc + sb ) / (2+k) + D*rLd ;
	      rLa  = rLc ;
  	    rLd  = rLb ;
      }
    }
  }

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  static
  void
  evalXYaSmall( valueType   a,
                valueType   b,
                indexType   p,
                valueType & X,
                valueType & Y ) {

    CLOTHOID_ASSERT( p < 11 && p > 0,
                     "In evalXYaSmall p = " << p << " must be in 1..10" ) ;

    valueType X0[43], Y0[43] ;

    indexType nkk = 4*p + 3 ; // max 43
    evalXYazero( nkk, b, X0, Y0 ) ;

    X = X0[0]-(a/2)*Y0[2] ;
    Y = Y0[0]+(a/2)*X0[2] ;

    valueType t  = 1 ;
    valueType aa = -a*a/4 ; // controllare!
    for ( indexType n=1 ; n <= p ; ++n ) {
      t *= aa/(2*n*(2*n-1)) ;
      valueType bf = a/(4*n+2) ;
      indexType jj = 4*n ;
      X += t*(X0[jj]-bf*Y0[jj+2]) ;
      Y += t*(Y0[jj]+bf*X0[jj+2]) ;
    }
  }

  // -------------------------------------------------------------------------

  static
  void
  evalXYaSmall( indexType nk,
                valueType a,
                valueType b,
                indexType p,
                valueType X[],
                valueType Y[] ) {

    indexType nkk = nk + 4*p + 2 ; // max 45
    valueType X0[45], Y0[45] ;

    CLOTHOID_ASSERT( nkk < 46,
                     "In evalXYaSmall (nk,p) = (" << nk << "," << p << ")\n" <<
                     "nk + 4*p + 2 = " << nkk  << " must be less than 46\n" ) ;

    evalXYazero( nkk, b, X0, Y0 ) ;

    for ( indexType j=0 ; j < nk ; ++j ) {
      X[j] = X0[j]-(a/2)*Y0[j+2] ;
      Y[j] = Y0[j]+(a/2)*X0[j+2] ;
    }

    valueType t  = 1 ;
    valueType aa = -a*a/4 ; // controllare!
    for ( indexType n=1 ; n <= p ; ++n ) {
      t *= aa/(2*n*(2*n-1)) ;
      valueType bf = a/(4*n+2) ;
      for ( indexType j = 0 ; j < nk ; ++j ) {
        indexType jj = 4*n+j ;
        X[j] += t*(X0[jj]-bf*Y0[jj+2]) ;
        Y[j] += t*(Y0[jj]+bf*X0[jj+2]) ;
      }
    }
  }
  
  //! \endcond

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  void
  GeneralizedFresnelCS( valueType   a,
                        valueType   b,
                        valueType   c,
                        valueType & intC,
                        valueType & intS ) {

    valueType xx, yy ;
    if ( std::abs(a) < A_THRESOLD ) evalXYaSmall( a, b, A_SERIE_SIZE, xx, yy ) ;
    else                            evalXYaLarge( a, b, xx, yy ) ;

    valueType cosc = cos(c) ;
    valueType sinc = sin(c) ;

    intC = xx * cosc - yy * sinc ;
    intS = xx * sinc + yy * cosc ;
  }

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  
  void
  GeneralizedFresnelCS( indexType nk,
                        valueType a,
                        valueType b,
                        valueType c,
                        valueType intC[],
                        valueType intS[] ) {

    CLOTHOID_ASSERT( nk > 0 && nk < 4, "nk = " << nk << " must be in 1..3" ) ;

    if ( std::abs(a) < A_THRESOLD ) evalXYaSmall( nk, a, b, A_SERIE_SIZE, intC, intS ) ;
    else                            evalXYaLarge( nk, a, b, intC, intS ) ;

    valueType cosc = cos(c) ;
    valueType sinc = sin(c) ;

    for ( indexType k = 0 ; k < nk ; ++k ) {
      valueType xx = intC[k] ;
      valueType yy = intS[k] ;
      intC[k] = xx * cosc - yy * sinc ;
      intS[k] = xx * sinc + yy * cosc ;
    }
  }

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------

  static valueType const CF[] = { 2.989696028701907,  0.716228953608281,
                                 -0.458969738821509, -0.502821153340377,
                                  0.261062141752652, -0.045854475238709 } ;

  int
  buildClothoid( valueType   x0,
                 valueType   y0,
                 valueType   theta0,
                 valueType   x1,
                 valueType   y1,
                 valueType   theta1,
                 valueType & k,
                 valueType & dk,
                 valueType & L ) {

    // traslazione in (0,0)
    valueType dx  = x1 - x0 ;
    valueType dy  = y1 - y0 ;
    valueType r   = hypot( dx, dy ) ;
    valueType phi = atan2( dy, dx ) ;

    valueType phi0 = theta0 - phi ;
    valueType phi1 = theta1 - phi ;
    
    phi0 -= m_2pi*round(phi0/m_2pi) ;
    phi1 -= m_2pi*round(phi1/m_2pi) ;

    if ( phi0 >  m_pi ) phi0 -= m_2pi ;
    if ( phi0 < -m_pi ) phi0 += m_2pi ;
    if ( phi1 >  m_pi ) phi1 -= m_2pi ;
    if ( phi1 < -m_pi ) phi1 += m_2pi ;

    valueType delta = phi1 - phi0 ;

    // punto iniziale
    valueType X  = phi0*m_1_pi ;
    valueType Y  = phi1*m_1_pi ;
    valueType xy = X*Y ;
    Y *= Y ; X *= X ;
    valueType A  = (phi0+phi1)*(CF[0]+xy*(CF[1]+xy*CF[2])+(CF[3]+xy*CF[4])*(X+Y)+CF[5]*(X*X+Y*Y)) ;

    // newton
    valueType g=0, dg, intC[3], intS[3] ;
    indexType niter = 0 ;
    do {
      GeneralizedFresnelCS( 3, 2*A, delta-A, phi0, intC, intS ) ;
      g   = intS[0] ;
      dg  = intC[2] - intC[1] ;
      A  -= g / dg ;
    } while ( ++niter <= 10 && std::abs(g) > 1E-12 ) ;

    CLOTHOID_ASSERT( std::abs(g) < 1E-8, "Newton do not converge, g = " << g << " niter = " << niter ) ;
    GeneralizedFresnelCS( 2*A, delta-A, phi0, intC[0], intS[0] ) ;
    L = r/intC[0] ;

    CLOTHOID_ASSERT( L > 0, "Negative length L = " << L ) ;
    k  = (delta-A)/L ;
    dk = 2*A/L/L ;
    
    return niter ;
  }

  int
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
                 valueType & L_2 ) {

    // traslazione in (0,0)
    valueType dx  = x1 - x0 ;
    valueType dy  = y1 - y0 ;
    valueType r   = hypot( dx, dy ) ;
    valueType phi = atan2( dy, dx ) ;

    valueType phi0 = theta0 - phi ;
    valueType phi1 = theta1 - phi ;
    
    phi0 -= m_2pi*round(phi0/m_2pi) ;
    phi1 -= m_2pi*round(phi1/m_2pi) ;

    if ( phi0 >  m_pi ) phi0 -= m_2pi ;
    if ( phi0 < -m_pi ) phi0 += m_2pi ;
    if ( phi1 >  m_pi ) phi1 -= m_2pi ;
    if ( phi1 < -m_pi ) phi1 += m_2pi ;

    valueType delta = phi1 - phi0 ;

    // punto iniziale
    valueType X  = phi0*m_1_pi ;
    valueType Y  = phi1*m_1_pi ;
    valueType xy = X*Y ;
    Y *= Y ; X *= X ;
    valueType A = (phi0+phi1)*(CF[0]+xy*(CF[1]+xy*CF[2])+(CF[3]+xy*CF[4])*(X+Y)+CF[5]*(X*X+Y*Y)) ;

    // newton
    valueType g=0, dg, intC[3], intS[3] ;
    indexType niter = 0 ;
    do {
      GeneralizedFresnelCS( 3, 2*A, delta-A, phi0, intC, intS ) ;
      g   = intS[0] ;
      dg  = intC[2] - intC[1] ;
      A  -= g / dg ;
    } while ( ++niter <= 10 && std::abs(g) > 1E-12 ) ;

    CLOTHOID_ASSERT( std::abs(g) < 1E-8, "Newton do not converge, g = " << g << " niter = " << niter ) ;
    GeneralizedFresnelCS( 3, 2*A, delta-A, phi0, intC, intS ) ;
    L = r/intC[0] ;

    CLOTHOID_ASSERT( L > 0, "Negative length L = " << L ) ;
    k  = (delta-A)/L ;
    dk = 2*A/L/L ;

    valueType alpha = intC[0]*intC[1] + intS[0]*intS[1] ;
    valueType beta  = intC[0]*intC[2] + intS[0]*intS[2] ;
    valueType gamma = intC[0]*intC[0] + intS[0]*intS[0] ;
    valueType tx    = intC[1]-intC[2] ;
    valueType ty    = intS[1]-intS[2] ;
    valueType txy   = L*(intC[1]*intS[2]-intC[2]*intS[1]) ;
    valueType omega = L*(intS[0]*tx-intC[0]*ty) - txy ;

    delta = intC[0]*tx + intS[0]*ty ;

    L_1  = omega/delta ;
    L_2  = txy/delta ;

    delta *= L ;
    k_1  = (beta-gamma-k*omega)/delta ;
    k_2  = -(beta+k*txy)/delta ;

    delta *= L/2 ;
    dk_1 = (gamma-alpha-dk*omega*L)/delta ;
    dk_2 = (alpha-dk*txy*L)/delta ;

    return niter ;
  }

  // ---------------------------------------------------------------------------

  void
  ClothoidCurve::eval( valueType   s,
                       valueType & theta,
                       valueType & kappa,
                       valueType & x,
                       valueType & y ) const {
    valueType C, S ;
    GeneralizedFresnelCS( dk*s*s, k*s, theta0, C, S ) ;
    x = x0 + s*C ;
    y = y0 + s*S ;
    theta = theta0 + s*(k+s*(dk/2)) ;
    kappa = k + s*dk ;
  }

  void
  ClothoidCurve::eval( valueType s, valueType & x, valueType & y ) const {
    valueType C, S ;
    GeneralizedFresnelCS( dk*s*s, k*s, theta0, C, S ) ;
    x = x0 + s*C ;
    y = y0 + s*S ;
  }

  void
  ClothoidCurve::eval_D( valueType s, valueType & x_D, valueType & y_D ) const {
    valueType theta = theta0 + s*(k+s*(dk/2)) ;
    x_D = cos(theta) ;
    y_D = sin(theta) ;
  }

  void
  ClothoidCurve::eval_DD( valueType s, valueType & x_DD, valueType & y_DD ) const {
    valueType theta   = theta0 + s*(k+s*(dk/2)) ;
    valueType theta_D = k+s*dk ;
    x_DD = -sin(theta)*theta_D ;
    y_DD =  cos(theta)*theta_D  ;
  }

  void
  ClothoidCurve::eval_DDD( valueType s, valueType & x_DDD, valueType & y_DDD ) const {
    valueType theta   = theta0 + s*(k+s*(dk/2)) ;
    valueType theta_D = k+s*dk ;
    valueType C       = cos(theta) ;
    valueType S       = sin(theta) ;
    valueType th2     = theta_D*theta_D ;
    x_DDD = -C*th2-S*dk ;
    y_DDD = -S*th2+C*dk  ;
  }

  // offset curve
  void
  ClothoidCurve::eval( valueType s, valueType offs, valueType & x, valueType & y ) const {
    valueType C, S ;
    GeneralizedFresnelCS( dk*s*s, k*s, theta0, C, S ) ;
    valueType theta = theta0 + s*(k+s*(dk/2)) ;
    valueType nx    = -sin(theta) ;
    valueType ny    =  cos(theta) ;
    x = x0 + s*C + offs * nx ;
    y = y0 + s*S + offs * ny ;
  }

  void
  ClothoidCurve::eval_D( valueType s, valueType offs, valueType & x_D, valueType & y_D ) const {
    valueType theta   = theta0 + s*(k+s*(dk/2)) ;
    valueType theta_D = k+s*dk ;
    valueType scale   = 1-offs*theta_D ;
    x_D = cos(theta)*scale ;
    y_D = sin(theta)*scale ;
  }

  void
  ClothoidCurve::eval_DD( valueType s, valueType offs, valueType & x_DD, valueType & y_DD ) const {
    valueType theta   = theta0 + s*(k+s*(dk/2)) ;
    valueType theta_D = k+s*dk ;
    valueType C       = cos(theta) ;
    valueType S       = sin(theta) ;
    valueType tmp1    = theta_D*(1-theta_D*offs) ;
    valueType tmp2    = offs*dk ;
    x_DD = -tmp1*S - C*tmp2 ;
    y_DD =  tmp1*C - S*tmp2 ;
  }

  void
  ClothoidCurve::eval_DDD( valueType s, valueType offs, valueType & x_DDD, valueType & y_DDD ) const {
    valueType theta   = theta0 + s*(k+s*(dk/2)) ;
    valueType theta_D = k+s*dk ;
    valueType C       = cos(theta) ;
    valueType S       = sin(theta) ;
    valueType tmp1    = theta_D*theta_D*(theta_D*offs-1) ;
    valueType tmp2    = dk*(1-3*theta_D*offs) ;
    x_DDD = tmp1*C-tmp2*S ;
    y_DDD = tmp1*S+tmp2*C ;
  }

  static
  valueType
  kappa( valueType theta0, valueType theta ) {
    valueType x = theta0*theta0 ;
    valueType a = -3.714 + x * 0.178 ;
    valueType b = -1.913 - x * 0.0753 ;
    valueType c =  0.999 + x * 0.03475 ;
    valueType d =  0.191 - x * 0.00703 ;
    valueType e =  0.500 - x * -0.00172 ;
    valueType t = d*theta0+e*theta ;
    return a*theta0+b*theta+c*(t*t*t) ;
  }

  static
  valueType
  theta_guess( valueType theta0, valueType k0, bool & ok ) {
    valueType x   = theta0*theta0 ;
    valueType a   = -3.714 + x * 0.178 ;
    valueType b   = -1.913 - x * 0.0753 ;
    valueType c   =  0.999 + x * 0.03475 ;
    valueType d   =  0.191 - x * 0.00703 ;
    valueType e   =  0.500 - x * -0.00172 ;
    valueType e2  = e*e ;
    valueType dt  = d*theta0 ;
    valueType dt2 = dt*dt ;
    valueType A   = c*e*e2 ;
    valueType B   = 3*(c*d*e2*theta0) ;
    valueType C   = 3*c*e*dt2 + b ;
    valueType D   = c*(dt*dt2) + a*theta0 - k0 ;

    valueType r[3] ;
    indexType nr, nc ;
    PolynomialRoots::solveCubic( A, B, C, D, r[0], r[1], r[2], nr, nc ) ;
    // cerco radice reale piu vicina
    valueType theta ;
    switch ( nr ) {
    case 0:
    default:
      ok = false ;
      return 0 ;
    case 1:
      theta = r[0] ;
      break ;
    case 2:
      if ( abs(r[0]-theta0) < abs(r[1]-theta0) ) theta = r[0] ;
      else                                       theta = r[1] ;
      break ;
    case 3:
      theta = r[0] ;
      for ( indexType i = 1 ; i < 3 ; ++i ) {
        if ( abs(theta-theta0) > abs(r[i]-theta0) )
          theta = r[i] ;
      }
      break ;
    }
    ok = abs(theta-theta0) < m_pi ;
    return theta ;
  }

  bool
  ClothoidCurve::setup_forward( valueType _x0,
                                valueType _y0,
                                valueType _theta0,
                                valueType _k,
                                valueType _x1,
                                valueType _y1,
                                valueType tol ) {

    x0     = _x0 ;
    y0     = _y0 ;
    theta0 = _theta0 ;
    k      = _k ;
    s_min  = 0 ;

    // Compute guess angles
    valueType len  = hypot( _y1-_y0, _x1-_x0 ) ;
    valueType arot = atan2( _y1-_y0, _x1-_x0 ) ;
    valueType th0  = theta0 - arot ;
    // normalize angle
    while ( th0 >  m_pi ) th0 -= m_2pi ;
    while ( th0 < -m_pi ) th0 += m_2pi ;

    // solve the problem from (0,0) to (1,0)
    valueType k0    = k*len ;
    valueType alpha = 2.6 ;
    valueType thmin = max(-m_pi,-theta0/2-alpha) ;
    valueType thmax = min( m_pi,-theta0/2+alpha) ;
    valueType Kmin  = kappa( th0, thmax ) ;
    valueType Kmax  = kappa( th0, thmin ) ;
    bool ok ;
    valueType th = theta_guess( th0, max(min(k0,Kmax),Kmin), ok ) ;
    if ( ok ) {
      for ( indexType iter = 0 ; iter < 10 ; ++iter ) {
        valueType dk, L, k_1, dk_1, L_1, k_2, dk_2, L_2 ;
        buildClothoid( 0, 0, th0,
                       1, 0, th,
                       k, dk, L, k_1, dk_1, L_1, k_2, dk_2, L_2 ) ;
        valueType f   = k - k0 ;
        valueType df  = k_2 ;
        valueType dth = f/df ;
        th -= dth ;
        if ( abs(dth) < tol && abs(f) < tol ) {
          // transform solution
          buildClothoid( x0, y0, theta0,
                         _x1, _y1, arot + th,
                         _k, dk, s_max ) ;
          return true ;
        }
      }
    }
    return false ;
  }

  void
  ClothoidCurve::change_origin( valueType s0 ) {
    valueType new_theta, new_kappa, new_x0, new_y0 ;
    eval( s0, new_theta, new_kappa, new_x0, new_y0 ) ;
    x0     = new_x0 ;
    y0     = new_y0 ;
    theta0 = new_theta ;
    k      = new_kappa ;
    s_min -= s0 ;
    s_max -= s0 ;
  }

  bool
  ClothoidCurve::bbTriangle( valueType offs,
                             valueType p0[2],
                             valueType p1[2],
                             valueType p2[2] ) const {
    valueType theta_max = theta( s_max ) ;
    valueType theta_min = theta( s_min ) ;
    valueType dtheta    = std::abs( theta_max-theta_min ) ;
    if ( dtheta < m_pi_2 ) {
      valueType alpha, t0[2] ;
      eval( s_min, offs, p0[0], p0[1] ) ;
      eval_D( s_min, t0[0], t0[1] ) ; // no offset
      if ( dtheta > 0.0001 * m_pi_2 ) {
        valueType t1[2] ;
        eval( s_max, offs, p1[0], p1[1] ) ;
        eval_D( s_max, t1[0], t1[1] ) ; // no offset
        // risolvo il sistema
        // p0 + alpha * t0 = p1 + beta * t1
        // alpha * t0 - beta * t1 = p1 - p0
        valueType det = t1[0]*t0[1]-t0[0]*t1[1] ;
        alpha = ((p1[1]-p0[1])*t1[0] - (p1[0]-p0[0])*t1[1])/det ;
      } else {
        // se angolo troppo piccolo uso approx piu rozza
        alpha = s_max - s_min ;
      }
      p2[0] = p0[0] + alpha*t0[0] ;
      p2[1] = p0[1] + alpha*t0[1] ;
      return true ;
    } else {
      return false ;
    }
  }

  void
  ClothoidCurve::bbSplit( valueType               split_angle,
                          valueType               split_size,
                          valueType               split_offs,
                          vector<ClothoidCurve> & c,
                          vector<Triangle2D>    & t ) const {

    // step 0: controllo se curvatura passa per 0
    valueType k_min = theta_D( s_min ) ;
    valueType k_max = theta_D( s_max ) ;
    c.clear() ;
    t.clear() ;
    if ( k_min * k_max < 0 ) {
      // risolvo (s-s_min)*dk+k_min = 0 --> s = s_min-k_min/dk
      valueType s_med = s_min-k_min/dk ;
      ClothoidCurve tmp(*this) ;
      tmp.trim(s_min,s_med) ;
      tmp.bbSplit_internal( split_angle, split_size, split_offs, c, t ) ;
      tmp.trim(s_med,s_max) ;
      tmp.bbSplit_internal( split_angle, split_size, split_offs, c, t ) ;
    } else {
      bbSplit_internal( split_angle, split_size, split_offs, c, t ) ;
    }
  }

  static
  valueType
  abs2pi( valueType a ) {
    a = std::abs(a) ;
    while ( a > m_pi ) a -= m_2pi ;
    return std::abs(a) ;
  }

  void
  ClothoidCurve::bbSplit_internal( valueType               split_angle,
                                   valueType               split_size,
                                   valueType               split_offs,
                                   vector<ClothoidCurve> & c,
                                   vector<Triangle2D>    & t ) const {

    valueType theta_min, kappa_min, x_min, y_min,
              theta_max, kappa_max, x_max, y_max ;

    eval( s_min, theta_min, kappa_min, x_min, y_min ) ;
    eval( s_max, theta_max, kappa_max, x_max, y_max ) ;

    valueType dtheta = std::abs( theta_max - theta_min ) ;
    valueType dx     = x_max - x_min ;
    valueType dy     = y_max - y_min ;
    valueType len    = hypot( dy, dx ) ;
    valueType dangle = abs2pi(atan2( dy, dx )-theta_min) ;
    if ( dtheta <= split_angle && len*tan(dangle) <= split_size ) {
      Triangle2D tt ;
      this->bbTriangle(split_offs,tt) ;
      c.push_back(*this) ;
      t.push_back(tt) ;
    } else {
      ClothoidCurve cc(*this) ;
      valueType s_med = (s_min+s_max)/2 ;
      cc.trim(s_min,s_med) ;
      cc.bbSplit_internal( split_angle, split_size, split_offs, c, t ) ;
      cc.trim(s_med,s_max) ;
      cc.bbSplit_internal( split_angle, split_size, split_offs, c, t ) ;
    }
  }

  bool
  ClothoidCurve::intersect_internal( ClothoidCurve & c1,
                                     valueType       c1_offs,
                                     valueType     & s1,
                                     ClothoidCurve & c2,
                                     valueType       c2_offs,
                                     valueType     & s2,
                                     indexType       max_iter,
                                     valueType       tolerance ) const {
    valueType angle1a = c1.theta(c1.s_min) ;
    valueType angle1b = c1.theta(c1.s_max) ;
    valueType angle2a = c2.theta(c2.s_min) ;
    valueType angle2b = c2.theta(c2.s_max) ;
    // cerca angoli migliori per partire
    valueType dmax = abs2pi(angle1a-angle2a) ;
    valueType dab  = abs2pi(angle1a-angle2b) ;
    valueType dba  = abs2pi(angle1b-angle2a) ;
    valueType dbb  = abs2pi(angle1b-angle2b) ;
    s1 = c1.s_min ; s2 = c2.s_min ;
    if ( dmax < dab ) { dmax = dab ; s2 = c2.s_max ; }
    if ( dmax < dba ) { dmax = dba ; s1 = c1.s_min ; s2 = c2.s_min ; }
    if ( dmax < dbb ) {              s1 = c1.s_min ; s2 = c2.s_max ; }
    for ( indexType i = 0 ; i < max_iter ; ++i ) {
      valueType t1[2], t2[2], p1[2], p2[2] ;
      c1.eval( s1, c1_offs, p1[0], p1[1] ) ;
      c1.eval_D( s1, c1_offs, t1[0], t1[1] ) ;
      c2.eval( s2, c2_offs, p2[0], p2[1] ) ;
      c2.eval_D( s2, c2_offs, t2[0], t2[1] ) ;
      /*
      // risolvo il sistema
      // p1 + alpha * t1 = p2 + beta * t2
      // alpha * t1 - beta * t2 = p2 - p1
      //
      //  / t1[0] -t2[0] \ / alpha \ = / p2[0] - p1[0] \
      //  \ t1[1] -t2[1] / \ beta  /   \ p2[1] - p1[1] /
      */
      valueType det = t2[0]*t1[1]-t1[0]*t2[1] ;
      valueType px  = p2[0]-p1[0] ;
      valueType py  = p2[1]-p1[1] ;
      s1 += (py*t2[0] - px*t2[1])/det ;
      s2 += (t1[0]*py - t1[1]*px)/det ;
      if ( s1 <= c1.s_min || s1 >= c1.s_max ||
           s2 <= c2.s_min || s2 >= c2.s_max ) break ;
      if ( std::abs(px) <= tolerance ||
           std::abs(py) <= tolerance ) return true ;
    }
    return false ;
  }

  void
  ClothoidCurve::intersect( valueType             offs,
                            ClothoidCurve const & clot,
                            valueType             clot_offs,
                            vector<valueType>   & s1,
                            vector<valueType>   & s2,
                            indexType             max_iter,
                            valueType             tolerance ) const {
    vector<ClothoidCurve> c0, c1 ;
    vector<Triangle2D>    t0, t1 ;
    bbSplit( m_pi/50, (s_max-s_min)/3, offs, c0, t0 ) ;
    clot.bbSplit( m_pi/50, (clot.s_max-clot.s_min)/3, clot_offs, c1, t1 ) ;
    s1.clear() ;
    s2.clear() ;
    for ( indexType i = 0 ; i < indexType(c0.size()) ; ++i ) {
      for ( indexType j = 0 ; j < indexType(c1.size()) ; ++j ) {
        if ( t0[i].overlap(t1[j]) ) {
          // uso newton per cercare intersezione
          valueType tmp_s1, tmp_s2 ;
          bool ok = intersect_internal( c0[i], offs,      tmp_s1,
                                        c1[j], clot_offs, tmp_s2,
                                        max_iter, tolerance ) ;
          if ( ok ) {
            s1.push_back(tmp_s1) ;
            s2.push_back(tmp_s2) ;
          }
        }
      }
    }
  }
  
  // collision detection
  bool
  ClothoidCurve::approsimate_collision( valueType             offs,
                                        ClothoidCurve const & clot,
                                        valueType             clot_offs,
                                        valueType             max_angle,
                                        valueType             max_size ) const {
    vector<ClothoidCurve> c0, c1 ;
    vector<Triangle2D>    t0, t1 ;
    bbSplit( max_angle, max_size, offs, c0, t0 ) ;
    clot.bbSplit( max_angle, max_size, clot_offs, c1, t1 ) ;
    for ( indexType i = 0 ; i < indexType(c0.size()) ; ++i ) {
      for ( indexType j = 0 ; j < indexType(c1.size()) ; ++j ) {
        if ( t0[i].overlap(t1[j]) ) return true ;
      }
    }
    return false ;
  }

  void
  ClothoidCurve::rotate( valueType angle, valueType cx, valueType cy ) {
    valueType dx  = x0 - cx ;
    valueType dy  = y0 - cy ;
    valueType C   = cos(angle) ;
    valueType S   = sin(angle) ;
    valueType ndx = C*dx - S*dy ;
    valueType ndy = C*dy + S*dx ;
    x0      = cx + ndx ;
    y0      = cy + ndy ;
    theta0 += angle ;
  }

  void
  ClothoidCurve::scale( valueType s ) {
    k     /= s ;
    dk    /= s*s ;
    s_min *= s ;
    s_max *= s ;
  }

  void
  ClothoidCurve::reverse() {
    theta0 = theta0 + m_pi ;
    if ( theta0 > m_pi ) theta0 -= 2*m_pi ;
    k     = -k ;
    valueType tmp = s_max ;
    s_max = -s_min ;
    s_min = -tmp ;
  }

  std::ostream &
  operator << ( std::ostream & stream, ClothoidCurve const & c ) {
    stream <<   "x0     = " << c.x0
           << "\ny0     = " << c.y0
           << "\ntheta0 = " << c.theta0
           << "\nk      = " << c.k
           << "\ndk     = " << c.dk
           << "\nL      = " << c.s_max-c.s_min
           << "\ns_min  = " << c.s_min
           << "\ns_max  = " << c.s_max
           << "\n" ;
    return stream ;
  }

  static
  inline
  bool
  power2( valueType a )
  { return a*a ; }

  // **************************************************************************

  static
  inline
  bool
  solve2x2( valueType const b[2],
            valueType       A[2][2],
            valueType       x[2] ) {
    // full pivoting
    indexType ij = 0 ;
    valueType Amax = std::abs(A[0][0]) ;
    valueType tmp  = std::abs(A[0][1]) ;
    if ( tmp > Amax ) { ij = 1 ; Amax = tmp ; }
    tmp = std::abs(A[1][0]) ;
    if ( tmp > Amax ) { ij = 2 ; Amax = tmp ; }
    tmp = std::abs(A[1][1]) ;
    if ( tmp > Amax ) { ij = 3 ; Amax = tmp ; }
    if ( Amax == 0 ) return false ;
    indexType i[] = { 0, 1 } ;
    indexType j[] = { 0, 1 } ;
    if ( (ij&0x01) == 0x01 ) { j[0] = 1 ; j[1] = 0 ; }
    if ( (ij&0x02) == 0x02 ) { i[0] = 1 ; i[1] = 0 ; }
    // apply factorization
    A[i[1]][j[0]] /= A[i[0]][j[0]] ;
    A[i[1]][j[1]] -= A[i[1]][j[0]]*A[i[0]][j[1]] ;
    // check for singularity
    valueType epsi = 1e-10 ;
    if ( std::abs( A[i[1]][j[1]] ) < epsi ) {
      // L^+ Pb
      valueType tmp = (b[i[0]] + A[i[1]][j[0]]*b[i[1]]) /
                      ( (1+power2(A[i[1]][j[0]]) ) *
                        ( power2(A[i[0]][j[0]])+power2(A[i[0]][j[1]]) ) ) ;
      x[j[0]] = tmp*A[i[0]][j[0]] ;
      x[j[1]] = tmp*A[i[0]][j[1]] ;
    } else { // non singular
      // L^(-1) Pb
      x[j[0]] = b[i[0]] ;
      x[j[1]] = b[i[1]]-A[i[1]][j[0]]*x[j[0]] ;
      // U^(-1) x
      x[j[1]] /= A[i[1]][j[1]] ;
      x[j[0]]  = (x[j[0]]-A[i[0]][j[1]]*x[j[1]])/A[i[0]][j[0]] ;
    }
    return true ;
  }

  // **************************************************************************

  void
  G2data::setup( valueType _x0,
                 valueType _y0,
                 valueType _theta0,
                 valueType _kappa0,
                 valueType _x1,
                 valueType _y1,
                 valueType _theta1,
                 valueType _kappa1 ) {

    x0     = _x0 ;
    y0     = _y0 ;
    theta0 = _theta0;
    kappa0 = _kappa0 ;
    x1     = _x1 ;
    y1     = _y1 ;
    theta1 = _theta1 ;
    kappa1 = _kappa1 ;

    // scale problem
    valueType dx = x1 - x0 ;
    valueType dy = y1 - y0 ;
    phi    = atan2( dy, dx ) ;
    Lscale = 2/hypot( dx, dy ) ;

    th0 = theta0 - phi ;
    th1 = theta1 - phi ;

    k0 = kappa0/Lscale ;
    k1 = kappa1/Lscale ;

    DeltaK     = k1 - k0 ;
    DeltaTheta = th1 - th0 ;
  }

  void
  G2data::setTolerance( valueType tol ) {
    CLOTHOID_ASSERT( tol > 0 && tol <= 0.1,
                     "setTolerance, tolerance = " << tol << " must be in (0,0.1]" ) ;
    tolerance = tol ;
  }

  void
  G2data::setMaxIter( int miter ) {
    CLOTHOID_ASSERT( miter > 0 && miter <= 1000,
                     "setMaxIter, maxIter = " << miter << " must be in [1,1000]" ) ;
    maxIter = miter ;
  }

  // **************************************************************************

  void
  G2solve2arc::evalA( valueType   alpha,
                      valueType   L,
                      valueType & A,
                      valueType & A_1,
                      valueType & A_2 ) const {
    valueType K  = k0+k1 ;
    valueType aK = alpha*DeltaK ;
    A   = alpha*(L*(aK-K)+2*DeltaTheta) ;
    A_1 = (2*aK-K)*L+2*DeltaTheta;
    A_2 = alpha*(aK-K) ;
  }

  void
  G2solve2arc::evalG( valueType alpha,
                      valueType L,
                      valueType th,
                      valueType k,
                      valueType G[2],
                      valueType G_1[2],
                      valueType G_2[2] ) const {

    valueType A, A_1, A_2, X[3], Y[3] ;
    evalA( alpha, L, A, A_1, A_2 ) ;
    valueType ak = alpha*k ;
    valueType Lk = L*k ;
    GeneralizedFresnelCS( 3, A, ak*L, th, X, Y );

    G[0]   = alpha*X[0] ;
    G_1[0] = X[0]-alpha*(Y[2]*A_1/2+Y[1]*Lk) ;
    G_2[0] =     -alpha*(Y[2]*A_2/2+Y[1]*ak) ;

    G[1]   = alpha*Y[0] ;
    G_1[1] = Y[0]+alpha*(X[2]*A_1/2+X[1]*Lk) ;
    G_2[1] =      alpha*(X[2]*A_2/2+X[1]*ak) ;

  }

  void
  G2solve2arc::evalFJ( valueType const vars[2],
                       valueType       F[2],
                       valueType       J[2][2] ) const {

    valueType alpha = vars[0] ;
    valueType L     = vars[1] ;
    valueType G[2], G_1[2], G_2[2] ;

    evalG( alpha, L, th0, k0, G, G_1, G_2 ) ;

    F[0]    = G[0] - 2/L ;       F[1]    = G[1] ;
    J[0][0] = G_1[0] ;           J[1][0] = G_1[1] ;
    J[0][1] = G_2[0] + 2/(L*L) ; J[1][1] = G_2[1] ;

    evalG( alpha-1, L, th1, k1, G, G_1, G_2 ) ;
    F[0]    -= G[0] ;   F[1]    -= G[1] ;
    J[0][0] -= G_1[0] ; J[1][0] -= G_1[1] ;
    J[0][1] -= G_2[0] ; J[1][1] -= G_2[1] ;
  }
  
  // ---------------------------------------------------------------------------

  bool
  G2solve2arc::solve() {
    valueType X[2] = { 0.5, 2 } ;
    bool converged = false ;
    for ( int i = 0 ; i < maxIter && !converged ; ++i ) {
      valueType F[2], J[2][2], d[2] ;
      evalFJ( X, F, J ) ;
      if ( !solve2x2( F, J, d ) ) break ;
      valueType lenF = hypot(F[0],F[1]) ;
      X[0] -= d[0] ;
      X[1] -= d[1] ;
      converged = lenF < tolerance ;
    }
    if ( converged ) converged = X[1] > 0 && X[0] > 0 && X[0] < 1 ;
    if ( converged ) buildSolution( X[0], X[1] ) ;
    return converged ;
  }
  
  // **************************************************************************

  void
  G2solve2arc::buildSolution( valueType alpha, valueType L ) {
    valueType beta = 1-alpha ;
    valueType LL   = L/Lscale ;
    valueType s0   = LL*alpha ;
    valueType s1   = LL*beta ;

    valueType tmp = k0*alpha+k1*beta-2*DeltaTheta/L ;
    
    valueType dk0 = -Lscale*(k0+tmp)/s0 ;
    valueType dk1 =  Lscale*(k1+tmp)/s1 ;

    S0.setup( x0, y0, theta0, kappa0, dk0,  0, s0 ) ;
    S1.setup( x1, y1, theta1, kappa1, dk1, -s1, 0 ) ;
    S1.change_origin( -s1 ) ;
  }

  // **************************************************************************
  
  void
  G2solve3arc::setup( valueType _x0,
                      valueType _y0,
                      valueType _theta0,
                      valueType _kappa0,
                      valueType _frac0,
                      valueType _x1,
                      valueType _y1,
                      valueType _theta1,
                      valueType _kappa1,
                      valueType _frac1 ) {
    G2data::setup( _x0, _y0, _theta0, _kappa0, _x1, _y1, _theta1, _kappa1 ) ;

    valueType tmp = 1/(2-(_frac0+_frac1)) ;
    alpha = _frac0*tmp ;
    beta  = _frac1*tmp ;

    gamma  = (1-alpha-beta)/4 ;
    gamma2 = gamma*gamma ;

    a0 = alpha*k0 ;
    b1 = beta*k1 ;

    valueType ab = alpha-beta ;

    dK0_0 = 2*alpha*DeltaTheta ;
    dK0_1 = -alpha*(k0+a0+b1) ;
    dK0_2 = -alpha*gamma*(beta-alpha+1) ;

    dK1_0 = -2*beta*DeltaTheta ;
    dK1_1 = beta*(k1+a0+b1) ;
    dK1_2 = beta*gamma*(beta-alpha-1) ;

    KM_0  = 2*gamma*DeltaTheta ;
    KM_1  = -gamma*(a0+b1) ;
    KM_2  = gamma2*(alpha-beta) ;

    thM_0 = (ab*DeltaTheta+(th0+th1))/2 ;
    thM_1 = (a0-b1-ab*(a0+b1))/4 ;
    thM_2 = (gamma*(2*ab*ab-alpha-beta-1))/8  ;
  }

  void
  G2solve3arc::evalFJ( valueType const vars[2],
                       valueType       F[2],
                       valueType       J[2][2] ) const {

    valueType eta  = vars[0] ;
    valueType zeta = vars[1] ;

    valueType dK0 = dK0_0 + eta*dK0_1 + zeta*dK0_2 ;
    valueType dK1 = dK1_0 + eta*dK1_1 + zeta*dK1_2 ;
    valueType KM  = KM_0  + eta*KM_1  + zeta*KM_2  ;
    valueType thM = thM_0 + eta*thM_1 + zeta*thM_2 ;

    valueType xa[3], ya[3], xb[3], yb[3], xM[3], yM[3], xP[3], yP[3] ;

    GeneralizedFresnelCS( 3, dK0, a0*eta,  th0, xa, ya );
    GeneralizedFresnelCS( 3, dK1, -b1*eta, th1, xb, yb );
    GeneralizedFresnelCS( 3, gamma2*zeta, -KM, thM, xM, yM );
    GeneralizedFresnelCS( 3, gamma2*zeta,  KM, thM, xP, yP );

    F[0] = alpha*xa[0] + beta*xb[0] + gamma*(xM[0]+xP[0]) - 2/eta ;
    F[1] = alpha*ya[0] + beta*yb[0] + gamma*(yM[0]+yP[0]) ;

    // D F[0] / D eta
    J[0][0] = - alpha*(ya[2]*dK0_1/2+ya[1]*a0)
              - beta*(yb[2]*dK1_1/2-yb[1]*b1)
              + gamma * ((yM[1]-yP[1])*KM_1-(yM[0]+yP[0])*thM_1)
              + 2/(eta*eta) ;

    // D F[0] / D zeta
    J[0][1] = - alpha*(ya[2]*dK0_2/2) - beta*(yb[2]*dK1_2/2)
              - gamma*( (yM[2]+yP[2])*gamma2/2+(yP[1]-yM[1])*KM_2+(yP[0]+yM[0])*thM_2 ) ;

    // D F[1] / D eta
    J[1][0] = alpha*(xa[2]*dK0_1/2+xa[1]*a0) +
              beta*(xb[2]*dK1_1/2-xb[1]*b1) +
              gamma * ((xP[1]-xM[1])*KM_1+(xM[0]+xP[0])*thM_1) ;

    // D F[1] / D zeta
    J[1][1] = alpha*(xa[2]*dK0_2/2) + beta*(xb[2]*dK1_2/2)
            + gamma * ( (xM[2]+xP[2])*gamma2/2+(xP[1]-xM[1])*KM_2+(xP[0]+xM[0])*thM_2 ) ;

  }
  
  // ---------------------------------------------------------------------------

  bool
  G2solve3arc::solve() {
    valueType X[2] = { 2, 0 } ; // eta, zeta
    bool converged = false ;
    for ( int i = 0 ; i < maxIter && !converged ; ++i ) {
      valueType F[2], J[2][2], d[2] ;
      evalFJ( X, F, J ) ;
      if ( !solve2x2( F, J, d ) ) break ;
      valueType lenF = hypot(F[0],F[1]) ;
      X[0] -= d[0] ;
      X[1] -= d[1] ;
      converged = lenF < tolerance ;
    }
    if ( converged ) converged = X[0] > 0 ; // eta > 0 !
    if ( converged ) buildSolution( X[0], X[1] ) ;
    return converged ;
  }
  
  void
  G2solve3arc::buildSolution( valueType eta, valueType zeta ) {

    valueType L0 = eta*alpha ;
    valueType L1 = eta*beta  ;
    valueType LM = eta*gamma ;

    valueType dkappaM  = zeta*gamma2 ; // /(eta*eta)*LM*LM ;
    valueType dkappa0A = dK0_0 + eta*dK0_1 + zeta*dK0_2 ;
    valueType dkappa1B = dK1_0 + eta*dK1_1 + zeta*dK1_2 ;
    valueType kappaM   = KM_0  + eta*KM_1  + zeta*KM_2  ;
    valueType thetaM   = thM_0 + eta*thM_1 + zeta*thM_2 ;

    valueType xa, ya, xmL, ymL ;
    GeneralizedFresnelCS( dkappa0A, k0*L0, th0, xa, ya );
    GeneralizedFresnelCS( dkappaM, -kappaM, thetaM, xmL, ymL );

    valueType xM = L0 * xa + LM * xmL - 1 ;
    valueType yM = L0 * ya + LM * ymL ;
    
    // rovescia scalatura
    L0 /= Lscale ;
    L1 /= Lscale ;
    LM /= Lscale ;

    dkappa0A /= L0*L0 ;
    dkappa1B /= L1*L1 ;
    dkappaM  /= LM*LM ;
    kappaM   /= LM ;

    S0.setup( x0, y0, theta0, kappa0, dkappa0A,  0, L0 ) ;
    S1.setup( x1, y1, theta1, kappa1, dkappa1B, -L1, 0 ) ;

    // la trasformazione inversa da [-1,1] a (x0,y0)-(x1,y1)
    // g(x,y) = RotInv(phi)*(1/lambda*[X;Y] - [xbar;ybar]) = [x;y]

    valueType C  = cos(phi) ;
    valueType S  = sin(phi) ;
    valueType dx = (xM+1)/Lscale ;
    valueType dy = yM/Lscale ;
    SM.setup( x0 + C * dx - S * dy, y0 + C * dy + S * dx,
              thetaM+phi, kappaM, dkappaM, -LM, LM ) ;

    //Sguess.setup_G1( x0_orig, y0_orig, theta0_orig,
    //                 x1_orig, y1_orig, theta1_orig ) ;

    S1.change_origin( -L1 ) ;
    SM.change_origin( -LM ) ;
  }

}
