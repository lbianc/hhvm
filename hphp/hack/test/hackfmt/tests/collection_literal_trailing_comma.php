<?hh

$vec = Vector { $foo };
$vec = Vector { $foo, };
$vec = Vector { $foo, $bar };
$vec = Vector { $foo, $bar, };

$vec = Vector { $fooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo };
$vec = Vector { $fooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo, };
$vec = Vector { $foooooooooooooooooooooooooooo, $baaaaaaaaaaaaaaaaaaaaaaaaaaaar };
$vec = Vector { $foooooooooooooooooooooooooooo, $baaaaaaaaaaaaaaaaaaaaaaaaaaaar, };

$vec = Vector { $foooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo };
$vec = Vector { $foooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo, };
$vec = Vector { $fooooooooooooooooooooooooooooooooooo, $baaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaar };
$vec = Vector { $fooooooooooooooooooooooooooooooooooo, $baaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaar, };

$vec = Vector { $foo /* foo */ };
$vec = Vector { $foo, /* foo */  };

$vec = Vector {
  $foo // foo
};
$vec = Vector {
  $foo, // foo
};
$vec = Vector {
  $foo // foo
  ,
};

$vec = Vector { /* leading comment */ $foooooooooooooooo /* trailing comment */ };
$vec = Vector { /* leading comment */ $foooooooooooooooo, /* trailing comment */  };

$vec = Vector {
  /* leading comment */
  $foooooooooooooooo
  /* trailing comment */
};
$vec = Vector {
  /* leading comment */
  $foooooooooooooooo,
  /* trailing comment */
};
$vec = Vector {
  /* leading comment */
  $foooooooooooooooo
  /* trailing comment */
  ,
};
