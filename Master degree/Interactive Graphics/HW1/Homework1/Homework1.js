"use strict";

// Everything is in this function, called at the end of this file
var MyHW1 = function() {

    var canvas;
    var gl;
    // Main definitions
    var modelViewMatrixLoc, projectionMatrixLoc, nMatrixLoc;
    var modelViewMatrix, modelViewMatrixCyl, projectionMatrix, nMatrix;
    var program;
    // Arrays for positions, normals, texture and tangents
    var positionsArray = [];
    var normalsArray = [];
    var texCoordsArray = [];
    var tangArray = [];

    // EXERCISE 1 ------------------------------------------------------------------------------------------------------------------------------
    // my solid has 26 faces => 26*6=156
    var numPositions = 156;
    // My solid vertices
    var vertices = [
        vec4(-0.5, -0.5,  0.5, 1.0),
        vec4(-0.5,  -0.5,  -0.5, 1.0),        
        vec4(0.5, -0.5, -0.5, 1.0),
        vec4(0.5, -0.5, 0.5, 1.0),

        vec4(-0.5,  0.5,  0.5, 1.0),
        vec4(0.5, 0.5,  0.5, 1.0),

        vec4(-0.25, -0.25, 0.75, 1.0),
        vec4(-0.25, 0.25, 0.75, 1.0),
        vec4(0.25, 0.25, 0.75, 1.0),
        vec4(0.25, -0.25, 0.75, 1.0),

        vec4(-0.5, 0.5, -0.5, 1.0),
        vec4(0.5, 0.5, -0.5, 1.0),
        vec4(-0.25, 0.8,0.25, 1.0),
        vec4(-0.25, 0.8, -0.25, 1.0),
        vec4(0.25, 0.8, -0.25, 1.0),
        vec4(0.25, 0.8,0.25, 1.0),

        vec4(0.8,  0.9, 0.25, 1.0),
        vec4(0.75,  -0.25, 0.10, 1.0),
        vec4(0.75, -0.25, -0.10, 1.0),
        vec4(0.8, 0.9, -0.25, 1.0),

        vec4(0.25,0.25,-0.75, 1.0),
        vec4(0.25,-0.25,-0.75, 1.0),
        vec4(-0.25,-0.25,-0.75, 1.0),
        vec4(-0.25,0.25,-0.75, 1.0),

        vec4(-0.9,0.25,0.25, 1.0),
        vec4(-0.9,0.25,-0.25, 1.0),
        vec4(-0.75,-0.25,-0.25, 1.0),
        vec4(-0.75,-0.25,0.25, 1.0)
    ];

    // My solid constructor
    function constructMyObject(){
        quad(0, 1, 2, 3);

        quad(4, 0, 6, 7);
        quad(4, 7, 8, 5);
        quad(5, 8, 9, 3);
        quad(0, 3, 9, 6);
        quad(7, 6, 9, 8);

        quad(13, 10, 4, 12);
        quad(12, 4, 5, 15);
        quad(15, 5, 11, 14);
        quad(14, 11, 10, 13);
        quad(13, 12, 15, 14);
        
        quad(5, 16, 19, 11);
        quad(5, 3, 17, 16);
        quad(3, 2, 18, 17);
        quad(11, 19, 18, 2);
        quad(16, 17, 18, 19);

        quad(10, 11, 20, 23);
        quad(11, 2, 21, 20);
        quad(21, 2, 1, 22);
        quad(10, 23, 22, 1);
        quad(20, 21, 22, 23);

        quad(4, 10, 25, 24);
        quad(10, 1, 26, 25);
        quad(1, 0, 27, 26);
        quad(0, 4, 24, 27);
        quad(24, 25, 26, 27);
    }

    // 4 points -> quad surface (two triangles)
    function quad(a, b, c, d){
        var t1 = subtract(vertices[b], vertices[a]);
        var t2 = subtract(vertices[c], vertices[b]);
        var normal = cross(t1, t2);
        normal = vec3(normal);

        positionsArray.push(vertices[a]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[0]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[b]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[1]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[c]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[2]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[a]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[0]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[c]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[2]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[d]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[3]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));
    }

    // EXERCISE 2 ------------------------------------------------------------------------------------------------------------------------------
    // flags controlled with buttons
    var flag_bar = false;
    var flag_rotation = false;
    var flag_orietation = false;
    // Rotation Axis controls
    var xAxis = 0, yAxis = 1, zAxis = 2;
    var axis = 0;
    var theta = vec3(0, 0, 0);
    // rotation speed
    var delay = 70;
    // BARYCENTER COMPUTATION
    var bar = vec4(0, 0, 0, 1.0);
    for (var i = 0; i < 28; i++) {
    bar = add(bar, vertices[i]);
    }
    bar = mult(bar, vec4(1/28, 1/28, 1/28, 1/28));

    // EXERCISE 3 ------------------------------------------------------------------------------------------------------------------------------
    // VIEW
    const at = vec3(0.0, 0.0, 0.0);
    const up = vec3(0.0, 1.0, 0.0);
    var radius = 3;
    var thetaCam = 0;
    var phi = 0;

    // PERSPECTIVE
    var near = 1.1;
    var far = 4;
    var fov = 50;  
    var aspect = 1;

    // EXERCISE 4 ------------------------------------------------------------------------------------------------------------------------------
    // LIGHT
    // Cylinder center is in (0.0, 0.63, 1.5)
    var ncylinder;
    var flag_light = 1.0;
    
    var lightPosition1 = vec4(0.0, 0.63, 1.5, 1.0);
    var lightAmbient1 = vec4(0.0633, 0.0633, 0.0633, 1.0);
    var lightDiffuse1 = vec4(0.33, 0.33, 0.33, 1.0);
    var lightSpecular1 = vec4(0.33, 0.33, 0.33, 1.0);

    var lightPosition2 = vec4(0.4, 0.63, 1.5, 1.0);
    var lightAmbient2 = vec4(0.0633, 0.0633, 0.0633, 1.0);
    var lightDiffuse2 = vec4(0.33, 0.33, 0.33, 1.0);
    var lightSpecular2 = vec4(0.33, 0.33, 0.33, 1.0);

    var lightPosition3 = vec4(-0.4, 0.63, 1.5, 1.0);
    var lightAmbient3 = vec4(0.0633, 0.0633, 0.0633, 1.0);
    var lightDiffuse3 = vec4(0.33, 0.33, 0.33, 1.0);
    var lightSpecular3 = vec4(0.33, 0.33, 0.33, 1.0);

    function constructCylinder(){
        var myCylinder = cylinder(72, 3, true);
        myCylinder.scale(0.1, 0.8, 0.1);
        myCylinder.rotate(90.0, [ 0, 0, 1]);
        myCylinder.translate(0.0, 0.63, 1.5);
        ncylinder = myCylinder.TriangleVertices.length;
        positionsArray = positionsArray.concat(myCylinder.TriangleVertices);
        // Arrays passed even if not used for the cylinder:
        // In my desing, the cylindrical light is always immobile
        // Per-vertex and per-fragment shading models and texture are dedicated ONLY to my solid.
        normalsArray = normalsArray.concat(myCylinder.TriangleNormals);
        texCoordsArray = texCoordsArray.concat(myCylinder.TextureCoordinates);
        tangArray = tangArray.concat(myCylinder.TangCoordinates);
    }
    
    // CYLINDER - START OF CODE FROM THE BOOK, source: "geometry.js" file
    function cylinder(numSlices, numStacks, caps){

        var slices = 36;
        if(numSlices) slices = numSlices;
        var stacks = 1;
        if(numStacks) stacks = numStacks;
        var capsFlag = true;
        if(caps==false) capsFlag = caps;
        
        var data = {};   
        var top = 0.5;
        var bottom = -0.5;
        var radius = 0.5;
        var topCenter = [0.0, top, 0.0];
        var bottomCenter = [0.0, bottom, 0.0];
        var sideColor = [1.0, 0.0, 0.0, 1.0];
        var topColor = [0.0, 1.0, 0.0, 1.0];
        var bottomColor = [0.0, 0.0, 1.0, 1.0];
        var cylinderVertexCoordinates = [];
        var cylinderNormals = [];
        var cylinderVertexColors = [];
        var cylinderTextureCoordinates = [];
        var cylinderTangCoordinates = []; // tangents addiction to fix Firefox's "problem"
        
        // side
        for(var j=0; j<stacks; j++) {
            var stop = bottom + (j+1)*(top-bottom)/stacks;
            var sbottom = bottom + j*(top-bottom)/stacks;
            var topPoints = [];
            var bottomPoints = [];
            var topST = [];
            var bottomST = [];
            for(var i =0; i<slices; i++) {
                var theta = 2.0*i*Math.PI/slices;
                topPoints.push([radius*Math.sin(theta), stop, radius*Math.cos(theta), 1.0]);
                bottomPoints.push([radius*Math.sin(theta), sbottom, radius*Math.cos(theta), 1.0]);
            };
            
            topPoints.push([0.0, stop, radius, 1.0]);
            bottomPoints.push([0.0,  sbottom, radius, 1.0]);
            
            for(var i=0; i<slices; i++) {
                var a = topPoints[i];
                var d = topPoints[i+1];
                var b = bottomPoints[i];
                var c = bottomPoints[i+1];
                var u = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];
                var v = [c[0]-b[0], c[1]-b[1], c[2]-b[2]];
            
                var normal = [
                u[1]*v[2] - u[2]*v[1],
                u[2]*v[0] - u[0]*v[2],
                u[0]*v[1] - u[1]*v[0]
                ];
            
                var mag = Math.sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2])
                normal = [normal[0]/mag, normal[1]/mag, normal[2]/mag];
                cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
                cylinderVertexColors.push(sideColor);
                cylinderNormals.push([normal[0], normal[1], normal[2]]);
                cylinderTextureCoordinates.push([(i+1)/slices, j*(top-bottom)/stacks]);
                // For each vertex I associate a (0, 0) tangent (obviously it is not the actual tangent but it is alright)
                // In fact, I only need this so that tangArray has the same dimension of other arrays (Firefox "problem")
                cylinderTangCoordinates.push(vec2(0, 0));
            
                cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
                cylinderVertexColors.push(sideColor);
                cylinderNormals.push([normal[0], normal[1], normal[2]]);
                cylinderTextureCoordinates.push([i/slices, (j-1)*(top-bottom)/stacks]);
                cylinderTangCoordinates.push(vec2(0, 0));
            
                cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
                cylinderVertexColors.push(sideColor);
                cylinderNormals.push([normal[0], normal[1], normal[2]]);
                cylinderTextureCoordinates.push([(i+1)/slices, (j-1)*(top-bottom)/stacks]);
                cylinderTangCoordinates.push(vec2(0, 0));
            
                cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
                cylinderVertexColors.push(sideColor);
                cylinderNormals.push([normal[0], normal[1], normal[2]]);
                cylinderTextureCoordinates.push([(i+1)/slices, j*(top-bottom)/stacks]);
                cylinderTangCoordinates.push(vec2(0, 0));
            
                cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
                cylinderVertexColors.push(sideColor);
                cylinderNormals.push([normal[0], normal[1], normal[2]]);
                cylinderTextureCoordinates.push([(i+1)/slices, (j-1)*(top-bottom)/stacks]);
                cylinderTangCoordinates.push(vec2(0, 0));
            
                cylinderVertexCoordinates.push([d[0], d[1], d[2], 1.0]);
                cylinderVertexColors.push(sideColor);
                cylinderNormals.push([normal[0], normal[1], normal[2]]);
                cylinderTextureCoordinates.push([(i+1)/slices, j*(top-bottom)/stacks]);
                cylinderTangCoordinates.push(vec2(0, 0));
            };
        };
        
        var topPoints = [];
        var bottomPoints = [];
        for(var i =0; i<slices; i++) {
            var theta = 2.0*i*Math.PI/slices;
            topPoints.push([radius*Math.sin(theta), top, radius*Math.cos(theta), 1.0]);
            bottomPoints.push([radius*Math.sin(theta), bottom, radius*Math.cos(theta), 1.0]);
        };
        topPoints.push([0.0, top, radius, 1.0]);
        bottomPoints.push([0.0,  bottom, radius, 1.0]);
        
        if(capsFlag) {
        
        //top
        for(i=0; i<slices; i++) {
        normal = [0.0, 1.0, 0.0];
        var a = [0.0, top, 0.0, 1.0];
        var b = topPoints[i];
        var c = topPoints[i+1];
        cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
        cylinderVertexColors.push(topColor);
        cylinderNormals.push(normal);
        cylinderTextureCoordinates.push([0, 1]);
        cylinderTangCoordinates.push(vec2(0, 0));
        
        cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
        cylinderVertexColors.push(topColor);
        cylinderNormals.push(normal);
        cylinderTextureCoordinates.push([0, 1]);
        cylinderTangCoordinates.push(vec2(0, 0));
        
        cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
        cylinderVertexColors.push(topColor);
        cylinderNormals.push(normal);
        cylinderTextureCoordinates.push([0, 1]);
        cylinderTangCoordinates.push(vec2(0, 0));
        };
        
        //bottom
        for(i=0; i<slices; i++) {
        normal = [0.0, -1.0, 0.0];
        var a = [0.0, bottom, 0.0, 1.0];
        var b = bottomPoints[i];
        var c = bottomPoints[i+1];
        cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
        cylinderVertexColors.push(bottomColor);
        cylinderNormals.push(normal);
        cylinderTextureCoordinates.push([0, 1]);
        cylinderTangCoordinates.push(vec2(0, 0));
        
        cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
        cylinderVertexColors.push(bottomColor);
        cylinderNormals.push(normal);
        cylinderTextureCoordinates.push([0, 1]);
        cylinderTangCoordinates.push(vec2(0, 0));
        
        cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
        cylinderVertexColors.push(bottomColor);
        cylinderNormals.push(normal);
        cylinderTextureCoordinates.push([0, 1]);
        cylinderTangCoordinates.push(vec2(0, 0));
        };
        
        };
        function translate(x, y, z){
        for(var i=0; i<cylinderVertexCoordinates.length; i++) {
            cylinderVertexCoordinates[i][0] += x;
            cylinderVertexCoordinates[i][1] += y;
            cylinderVertexCoordinates[i][2] += z;
        };
        }
        
        function scale(sx, sy, sz){
            for(var i=0; i<cylinderVertexCoordinates.length; i++) {
                cylinderVertexCoordinates[i][0] *= sx;
                cylinderVertexCoordinates[i][1] *= sy;
                cylinderVertexCoordinates[i][2] *= sz;
                cylinderNormals[i][0] /= sx;
                cylinderNormals[i][1] /= sy;
                cylinderNormals[i][2] /= sz;
            };
        }
        
        function radians( degrees ) {
            return degrees * Math.PI / 180.0;
        }
        
        function rotate( angle, axis) {
            var d = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        
            var x = axis[0]/d;
            var y = axis[1]/d;
            var z = axis[2]/d;
        
            var c = Math.cos( radians(angle) );
            var omc = 1.0 - c;
            var s = Math.sin( radians(angle) );
        
            var mat = [
                [ x*x*omc + c,   x*y*omc - z*s, x*z*omc + y*s ],
                [ x*y*omc + z*s, y*y*omc + c,   y*z*omc - x*s ],
                [ x*z*omc - y*s, y*z*omc + x*s, z*z*omc + c ]
            ];
        
            for(var i=0; i<cylinderVertexCoordinates.length; i++) {
                var u = [0, 0, 0];
                var v = [0, 0, 0];
                for( var j =0; j<3; j++)
                for( var k =0 ; k<3; k++) {
                    u[j] += mat[j][k]*cylinderVertexCoordinates[i][k];
                    v[j] += mat[j][k]*cylinderNormals[i][k];
                    };
                for( var j =0; j<3; j++) {
                    cylinderVertexCoordinates[i][j] = u[j];
                    cylinderNormals[i][j] = v[j];
                };
            };
        }
        
        data.TriangleVertices = cylinderVertexCoordinates;
        data.TriangleNormals = cylinderNormals;
        data.TriangleVertexColors = cylinderVertexColors;
        data.TextureCoordinates = cylinderTextureCoordinates;
        data.TangCoordinates = cylinderTangCoordinates;
        data.rotate = rotate;
        data.translate = translate;
        data.scale = scale;
        return data;
    }
    // CYLINDER - END OF CODE FROM THE BOOK, source: "geometry.js" file

    // EXERCISE 5 ------------------------------------------------------------------------------------------------------------------------------
    // MATERIAL
    var materialAmbient, materialDiffuse, materialSpecular, materialShininess;
    var ambientProduct1, diffuseProduct1, specularProduct1;
    var ambientProduct2, diffuseProduct2, specularProduct2;
    var ambientProduct3, diffuseProduct3, specularProduct3;
    function setCylinderMaterial(){
        // new materialAmbient for the cylinder, just to give it some different attributes
        materialAmbient = vec4(1, 1, 1, 1.0);
        setMaterial();
    }

    function setSolidMaterial(){
        materialAmbient = vec4(0.5, 0.5, 0.5, 1.0);
        materialDiffuse = vec4(0.5, 0.5, 0.5, 1.0);
        materialSpecular = vec4(1.0, 1.0, 1.0, 1.0);
        materialShininess = 200.0;        
        setMaterial();
    }

    function setMaterial(){
        ambientProduct1 = mult(lightAmbient1, materialAmbient);
        diffuseProduct1 = mult(lightDiffuse1, materialDiffuse);
        specularProduct1 = mult(lightSpecular1, materialSpecular);

        ambientProduct2 = mult(lightAmbient2, materialAmbient);
        diffuseProduct2 = mult(lightDiffuse2, materialDiffuse);
        specularProduct2 = mult(lightSpecular2, materialSpecular);

        ambientProduct3 = mult(lightAmbient3, materialAmbient);
        diffuseProduct3 = mult(lightDiffuse3, materialDiffuse);
        specularProduct3 = mult(lightSpecular3, materialSpecular);

        gl.uniform1f(gl.getUniformLocation(program,"uShininess"), materialShininess);
        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct1"), ambientProduct1);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct1"), diffuseProduct1 );
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct1"), specularProduct1 );

        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct2"), ambientProduct2);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct2"), diffuseProduct2 );
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct2"), specularProduct2 );

        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct3"), ambientProduct3);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct3"), diffuseProduct3 );
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct3"), specularProduct3 );
        gl.uniform1f(gl.getUniformLocation(program,"uShininess"), materialShininess);
    }

    // EXERCISE 7 ------------------------------------------------------------------------------------------------------------------------------
    // TEXTURE
    var texSize = 32;
    var texCoord = [
        vec2(0, 0),
        vec2(0, 1),
        vec2(1, 1),
        vec2(1, 0)
    ];
    // texture flag
    var flag_texture = 0.0
    // TEXTURE CONSTRUCTION
    // Bump Data
    var data = new Array()
    for (var i = 0; i<= texSize; i++)  data[i] = new Array();
    for (var i = 0; i<= texSize; i++) for (var j=0; j<=texSize; j++)
        data[i][j] = Math.random()*2
    // Bump Map Normals
    var normalst = new Array()
    for (var i=0; i<texSize; i++)  normalst[i] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++)
        normalst[i][j] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++) {
        normalst[i][j][0] = data[i][j]-data[i+1][j];
        normalst[i][j][1] = data[i][j]-data[i][j+1];
        normalst[i][j][2] = 1;
    }
    // Scale to Texture Coordinates
    for (var i=0; i<texSize; i++) for (var j=0; j<texSize; j++) {
        var d = 0;
        for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
        d = Math.sqrt(d);
        for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
    }
    // Normal Texture Array
    var normals = new Uint8Array(3*texSize*texSize);
    for ( var i = 0; i < texSize; i++ )
        for ( var j = 0; j < texSize; j++ )
            for(var k =0; k<3; k++)
                normals[3*texSize*i+3*j+k] = 255*normalst[i][j][k];
    // texture -> applied
    function configureTexture(image) {
        var tex = gl.createTexture();
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, image);
        gl.generateMipmap(gl.TEXTURE_2D);
    }

    // INIT FUNCTION ------------------------------------------------------------------------------------------------------------------------------
    function init() {
        canvas = document.getElementById("gl-canvas");

        gl = canvas.getContext('webgl2');
        if (!gl) alert( "WebGL 2.0 isn't available");

        gl.viewport(0, 0, canvas.width, canvas.height);
        gl.clearColor(1.0, 1.0, 1.0, 1.0);
        gl.enable(gl.DEPTH_TEST);

        //  Load shaders and initialize attribute buffers
        program = initShaders(gl, "vertex-shader", "fragment-shader");
        gl.useProgram(program);
        constructMyObject();
        constructCylinder()

        var nBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);
        var normalLoc = gl.getAttribLocation(program, "aNormal");
        gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(normalLoc);

        var vBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(positionsArray), gl.STATIC_DRAW);
        var positionLoc = gl.getAttribLocation(program, "aPosition");
        gl.vertexAttribPointer(positionLoc, 4, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(positionLoc);

        var tBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);
        var textureLoc = gl.getAttribLocation(program, "aTexCoord");
        gl.vertexAttribPointer(textureLoc, 2, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(textureLoc);

        var tanBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, tanBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(tangArray), gl.STATIC_DRAW);
        var tangentLoc = gl.getAttribLocation(program, "aTangent");
        gl.vertexAttribPointer(tangentLoc, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(tangentLoc);
        
        // Apply texture
        configureTexture(normals);
        
        // BUTTONS AND SLIDERS
        document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
        document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
        document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};
        document.getElementById("ButtonT").onclick = function(){flag_rotation = !flag_rotation;};
        document.getElementById("ButtonB").onclick = function(){flag_bar = !flag_bar;};
        document.getElementById("ButtonO").onclick = function(){flag_orietation = !flag_orietation;};
        document.getElementById("ButtonF").onclick = function(){delay /= 1.5;};
        document.getElementById("ButtonS").onclick = function(){delay *= 1.5;};
        // Shader toggled (1.0)     -> per-vertex shading mode
        // Shader not toggled (0.0) -> per-fragment shading mode
        var flag_shader = 1.0;
        document.getElementById("ButtonShader").onclick = function(){
            if (flag_shader > 0.5)
                flag_shader = 0.0;
            else if(flag_shader < 0.5)
                flag_shader = 1.0;
            gl.uniform1f(gl.getUniformLocation(program, "uShader"), flag_shader);};
        
        document.getElementById("ButtonLight").onclick = function(){
            if(flag_light > 0.5)
                flag_light = 0.0;
            else if(flag_light < 0.5)
                flag_light = 1.0;
            gl.uniform1f(gl.getUniformLocation(program, "uLight"), flag_light);};

        document.getElementById("ButtonTexture").onclick = function(){
            if(flag_texture > 0.5)
                flag_texture = 0.0;
            else if(flag_texture < 0.5)
                flag_texture = 1.0;
            gl.uniform1f(gl.getUniformLocation(program, "uTexture"), flag_texture);};

        document.getElementById("radiusSlider").onchange = function(event) {radius = event.target.value;};
        document.getElementById("thetaSlider").onchange = function(event) {thetaCam = event.target.value* Math.PI/180.0;};
        document.getElementById("phiSlider").onchange = function(event) {phi = event.target.value* Math.PI/180.0;};
        document.getElementById("farSlider").onchange = function(event) {far = event.target.value;};
        document.getElementById("nearSlider").onchange = function(event) {near = event.target.value;};
        document.getElementById("aspectSlider").onchange = function(event) {aspect = event.target.value;};
        document.getElementById("fovSlider").onchange = function(event) {fov = event.target.value;};

        // Texture, lights and flags sent
        gl.uniform1i( gl.getUniformLocation(program, "uTextureMap"), 0);

        gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition1"), lightPosition1 );
        gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition2"), lightPosition2 );
        gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition3"), lightPosition3 );

        gl.uniform1f(gl.getUniformLocation(program, "uShader"), flag_shader);
        gl.uniform1f(gl.getUniformLocation(program, "uLight"), flag_light);
        gl.uniform1f(gl.getUniformLocation(program, "uTexture"), flag_texture);

        modelViewMatrixLoc = gl.getUniformLocation(program, "uModelViewMatrix");
        projectionMatrixLoc = gl.getUniformLocation(program, "uProjectionMatrix");
        nMatrixLoc = gl.getUniformLocation(program, "uNormalMatrix");

        render();
    }

    // RENDER FUNCTION ------------------------------------------------------------------------------------------------------------------------------
    function render(){
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
        // Rotation angle displacement
        if(flag_rotation){
            if(flag_orietation)
                theta[axis] -= 2.0;
            else
                theta[axis] += 2.0;
        }
        var eye = vec3(radius*Math.sin(thetaCam)*Math.cos(phi), radius*Math.sin(thetaCam)*Math.sin(phi), radius*Math.cos(thetaCam));
        modelViewMatrix = lookAt(eye, at, up);
        modelViewMatrixCyl = lookAt(eye, at, up);
        projectionMatrix = perspective(fov, aspect, near, far);
        
        // ROTATION AROUND BARYCENTER
        if(flag_bar){
            modelViewMatrix = mult(modelViewMatrix, translate(bar[0], bar[1], bar[2]));
            modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0)));
            modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0)));
            modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1)));
            modelViewMatrix = mult(modelViewMatrix, translate(-bar[0], -bar[1], -bar[2]));
        } 
        // ROTATION AROUND ORIGIN
        else{
            modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0)));
            modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0)));
            modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1)));
        }

        nMatrix = normalMatrix(modelViewMatrix, true )
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(modelViewMatrix));
        gl.uniformMatrix4fv(gl.getUniformLocation(program, "uModelViewMatrixCyl"), false, flatten(modelViewMatrixCyl));
        gl.uniformMatrix4fv(projectionMatrixLoc, false, flatten(projectionMatrix));
        gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));
        
        // MY SOLID
        var cyl = 0.0;
        gl.uniform1f(gl.getUniformLocation(program, "uCyl"), cyl);
        setSolidMaterial();
        gl.drawArrays(gl.TRIANGLES, 0, numPositions);

        // CYLINDER
        var cyl = 1.0;
        gl.uniform1f(gl.getUniformLocation(program, "uCyl"), cyl);
        setCylinderMaterial();
        gl.drawArrays( gl.TRIANGLES, numPositions, ncylinder);

        // render() called again after delay ms
        setTimeout(function (){requestAnimationFrame(render);}, delay);
    }

    init();

}

// Here the execution starts
MyHW1();