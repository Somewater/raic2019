#include "TreeNodeT.h"

std::ostream& operator<<(std::ostream& stream, const Evaluation& e) {
  return stream << '{' << e.value << '}';
}
