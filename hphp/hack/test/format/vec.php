<?hh
// Copyright 2004-present Facebook. All Rights Reserved.

$vec = vec [ ] ;
$vec = vec [ 1 ] ;
$vec = vec [ '1' ] ;
$vec = vec [ '1', 2 ] ;
$vec = vec [ vec [ vec [] ] ];

$vec = vec ( );
$vec = vec ( $vec );

foo() -> vec ( );
foo() -> vec ( $vec );

Foo::vec ( );
Foo::vec ( $vec );

$vec = Vec\sort ( $vec );
