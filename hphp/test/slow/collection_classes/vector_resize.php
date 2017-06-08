<?hh

function test($v, $s) {
  echo "---- from " . $v->count() . " to $s\n";
  $v->resize($s, 0);
  foreach ($v as $val) {
    var_dump($val);
  }
}

function main() {
  $vectors = array(
    Vector {},
    Vector {1},
    Vector {1, 2},
    Vector {1, 2, 3, 4, 5},
  );
  $sizes = array(0, 1, 3, 6);
  foreach ($vectors as $vec) {
    foreach ($sizes as $s) {
      test(new Vector($vec), $s);
    }
  }
}

main();
