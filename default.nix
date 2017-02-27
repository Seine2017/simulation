{ fetchurl, ode, python27Packages }:

let
  pyode = python27Packages.buildPythonPackage rec {
    name = "python-pyode-${version}";
    version = "1.2.0";

    src = fetchurl {
      url = "mirror://sourceforge/project/pyode/pyode/${version}/PyODE-${version}.tar.gz";
      sha256 = "1dwfkjj7a4n8xhfqrcyhmmb8xqhi0rs7m38p8sbfbk8y7a3qq7yq";
    };

    CPPFLAGS = ["-DdSINGLE"];

    buildInputs = [
      ode
    ];
  };
in
  python27Packages.buildPythonApplication rec {
    name = "simulation-${version}";
    version = "0.1";

    src = ./.;

    propagatedBuildInputs = with python27Packages; [
      pyode
    ];

    doCheck = false;
  }
