 /**
  Name: mapGeneric.h
  Purpose: General typed range mapper.

  @version: 0.1
*/

template <class X, class M, class N, class O, class Q>
X mapGeneric(X x, M inMin, N inMax, O outMin, Q outMax){
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
