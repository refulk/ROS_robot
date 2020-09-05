
//import * as myModule from 'Dados/historico.js';
var vetTarefas = [];
var imgIndex = 0;
var cargaArray = [0,3,5,8]; //pontos de carga
var descargaArray = [0,3,5,8]; //pontos de descarga

//Controla a altura do embedGNU automaticamente
//Utiliza o Iframe para saber a altura
function resizeIframe(obj) {
    document.getElementById('embedGNU').style.height = obj.contentWindow.document.documentElement.scrollHeight + 'px';
    obj.style.visibility = "hidden"; 
    obj.style.height = '0px';
  }

//executa após carregar a janela HTML
window.onload = function() 
{
	loadCmbCarga();	
    refreshImgVideo();
	window.setInterval(function() {
    	refreshImgVideo();
	}, 50);
};	

function refreshImgVideo()
{
	// create a new timestamp     
	var timestamp = new Date().getTime();  
	var file = "Img/imgVideo.png?t=" + timestamp;
	var img = new Image();
	img.src = file;
	img.onload = function() {
		document.getElementById("imgVideo").src = file;
	}
}		

function loadCmbCarga()
{
	var cmbCarga = document.getElementById('cmbCarga');
	for(var i = 0; i < cargaArray.length; i++)
	{
	   var t = document.createElement("option")
	   t.value = cargaArray[i];
	   t.textContent = cargaArray[i].toString();
	   cmbCarga.appendChild(t)
	}	
}

function limparComboBox(elemento, mascara, desabilitado)
{
	var length = elemento.options.length;
	for (i = length-1; i >= 0; i--) 
	{
	  elemento.options[i] = null;
	}
	if(mascara != "")
	{
		var t = document.createElement("option")
		t.value = "";
		t.textContent = mascara;
		t.disabled = false;
		elemento.appendChild(t);
		elemento.options[elemento.selectedIndex].disabled = true;	
	}
	elemento.disabled = desabilitado;
}

function loadCmbDescarga(valorCarga)
{
	var cmbDescarga = document.getElementById('cmbDescarga');
	limparComboBox(cmbDescarga,"Ponto de Descarga",true);
	var t = document.createElement("option")
	for(var i = 0; i < descargaArray.length; i++)
	{
		if(descargaArray[i] != valorCarga)
		{
			var t = document.createElement("option")
			t.value = descargaArray[i];
			t.textContent = descargaArray[i].toString();
			cmbDescarga.appendChild(t)			
		}
	}	
	cmbDescarga.disabled = false;
}

function funcCarga(elemento)
{
	btnAddTarefa.disabled=true;
	var valorCarga = elemento.options[elemento.selectedIndex].value;
	loadCmbDescarga(valorCarga);
	if(vetTarefas.length == 0)
		exibirTarefas();
}

function funcDescarga()
{
	btnAddTarefa.disabled=false;
}

function exibirTarefas()
{
	var rows = vetTarefas.length + 1;
    var tabela = document.getElementById('tabTarefas');    
    var text = "";    
	tabela.innerHTML = ""; //Limpar tabela
    if(vetTarefas.length == 0)
    {
        text = "";             
    }
    else
    {
        for (var j=0; j < vetTarefas.length; j++)
        {
            text += vetTarefas[j];            
        }
    }
    if(rows > 8)
    	rows = 8;
    tabela.rows = rows.toString();
    tabela.value = text;
}

function funcbtnAddTarefa()
{
	var cmbCarga = document.getElementById('cmbCarga');
	var valorCarga = cmbCarga.options[cmbCarga.selectedIndex].value;

	var cmbDescarga = document.getElementById('cmbDescarga');
	var valorDescarga = cmbDescarga.options[cmbDescarga.selectedIndex].value;

	vetTarefas.push("Carga:"+valorCarga+";Descarga:"+valorDescarga+";\n");
	btnLimparTarefa.disabled=false;
	btnExecutar.disabled=false;

	exibirTarefas();
}

function funcbtnLimparTarefa()
{
	document.getElementById("tabTarefas").value = ""; //Clean Table
	funcLimpar();
}

function funcLimpar()
{
	vetTarefas = [];
	btnAddTarefa.disabled=true;
	btnLimparTarefa.disabled=true;
	btnExecutar.disabled=true;
	resetarComboBoxes();
	loadCmbCarga();	
}

function resetarComboBoxes()
{
	limparComboBox(document.getElementById('cmbCarga'),"Ponto de Carga",false);
	limparComboBox(document.getElementById('cmbDescarga'),"Ponto de Descarga",true);	
}

function funcbtnExecutar()
{
  	alert("Tarefa enviada com sucesso!");
	funcLimpar();
}

function msg() {
  alert("Você clicou no botão!");
}
