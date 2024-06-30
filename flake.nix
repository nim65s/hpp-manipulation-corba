{
  description = "Corba server for manipulation planning";

  nixConfig = {
    extra-substituters = [ "https://gepetto.cachix.org" ];
    extra-trusted-public-keys = [ "gepetto.cachix.org-1:toswMl31VewC0jGkN6+gOelO2Yom0SOHzPwJMY2XiDY=" ];
  };

  inputs = {
    nixpkgs.url = "github:nim65s/nixpkgs/gepetto";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    hpp-corbaserver = {
      url = "github:humanoid-path-planner/hpp-corbaserver/release/5.1.0";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
        hpp-core.follows = "hpp-manipulation-urdf/hpp-manipulation/hpp-core";
        hpp-util.follows = "hpp-manipulation-urdf/hpp-manipulation/hpp-core/hpp-constraints/hpp-util";
      };
    };
    hpp-manipulation-urdf = {
      url = "github:humanoid-path-planner/hpp-manipulation-urdf/release/5.1.0";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
      };
    };
  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ ];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          self',
          pkgs,
          system,
          ...
        }:
        {
          packages = {
            inherit (pkgs) cachix;
            default = pkgs.callPackage ./. {
              hpp-corbaserver = inputs.hpp-corbaserver.packages.${system}.default;
              hpp-manipulation-urdf = inputs.hpp-manipulation-urdf.packages.${system}.default;
            };
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
        };
    };
}
